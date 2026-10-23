"""Framing, correlation and backpressure regression tests; no real hardware."""
import importlib.util
import sys
import threading
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock
import pytest
from miniarm_core.client import MiniArmClient

ROOT = Path(__file__).resolve().parents[1]

class Port:
    def __init__(self, incoming=b''):
        self.buffer = bytearray(incoming)
        self.is_open = True
        self.timeout = .01
        self.reads = 0
        self.writes = []
        self.respond = True
        self.payload = b'[12.2][Controller] [1, 2, 3, 4, 5, 6, 7]\r\n'
    @property
    def in_waiting(self): return len(self.buffer)
    def read(self, count):
        self.reads += 1
        result = bytes(self.buffer[:count])
        del self.buffer[:count]
        return result
    def write(self, data):
        self.writes.append(data)
        if self.respond:
            begin, command, end, _ = data.decode().split(';')
            self.buffer.extend(b'old unsolicited output\r\n')
            self.buffer.extend(('@'+begin+'\r\n').encode()+self.payload+('@'+end+'\r\n').encode())
        return len(data)


def reader(monkeypatch, incoming):
    port = Port(incoming)
    for name in ('board', 'busio'):
        monkeypatch.setitem(sys.modules, name, SimpleNamespace())
    monkeypatch.setitem(sys.modules, 'usb_cdc', SimpleNamespace(console=port))
    spec = importlib.util.spec_from_file_location('usb_reader_test', ROOT/'firmware/circuitpython/lib/arm_utils/usbserialreader.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module.USBSerialReader(use_UART=False, TERMINATOR=';'), port


def drain(reader, port):
    commands = []
    for _ in range(100):
        reader.update()
        if reader._out_data is not None: commands.extend(reader.out_data)
        if not port.in_waiting and not reader._pending: break
    return commands


def test_burst_keeps_order_and_reads_chunks(monkeypatch):
    r, p = reader(monkeypatch, b'test;get_joints;sync:abc-e;partial')
    assert drain(r,p) == [['test'], ['get_joints'], ['sync','abc-e']]
    assert p.reads == 1
    p.buffer.extend(b'_command;')
    assert drain(r,p) == [['partial_command']]


def test_overflow_and_invalid_encoding_never_execute_truncated_frame(monkeypatch):
    r,p = reader(monkeypatch, b'x'*513+b'home;\xff;test;')
    assert drain(r,p) == [['test']]
    assert len(r.in_data) == 0


def client(port):
    c = MiniArmClient.__new__(MiniArmClient)
    c.s = port; c.timeout = .01; c._io_lock = threading.RLock()
    c.verbose = False; c._last_joints = None
    return c


def test_complete_response_excludes_old_output_and_parses_logged_joints():
    p = Port(); c = client(p)
    assert c.get_joints() == [1,2,3,4,5,6,7]
    assert len(p.writes) == 1


def test_fragmented_response_and_multiple_lines_are_complete():
    p=Port(); original=p.read
    p.read=lambda n: original(min(n,3))
    p.payload=b'first\r\nsecond\r\n'
    assert client(p).send('info') == 'first\nsecond'


def test_timeout_does_not_retry_motion_or_accept_stale_fence():
    p=Port(b'@sync:wrong-e\r\n'); p.respond=False
    with pytest.raises(TimeoutError): client(p).send('home')
    assert len(p.writes)==1
    assert p.timeout == .01


def test_firmware_error_is_not_success():
    p=Port();p.payload=b'Unknown error\r\n'
    with pytest.raises(RuntimeError): client(p).send('test')


@pytest.mark.parametrize('command', ['home;home', 'test\n', 'sync:abc', 'x'*513])
def test_invalid_host_frame_never_written(command):
    p=Port()
    with pytest.raises(ValueError): client(p).send(command)
    assert not p.writes


def test_circuitpython_terminal_title_cannot_hide_reply_fence():
    p=Port();original=p.write
    def write(data):
        result=original(data)
        prefix=b'\x1b]0;Wi-Fi: off | code.py | 9.2.4\x1b\\'
        p.buffer=bytearray(bytes(p.buffer).replace(b'@sync:',prefix+b'@sync:'))
        return result
    p.write=write
    assert client(p).send('test') == '[12.2][Controller] [1, 2, 3, 4, 5, 6, 7]'
