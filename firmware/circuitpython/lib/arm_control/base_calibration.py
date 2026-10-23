"""Bounded independent servo desk tests; no pose/encoder inference.

Mini-Arm contributors: Jonathan Shulgach; implementation assisted by OpenAI Codex.
See the project's existing license and attribution requirements.
"""
import math
import time


class BaseCalibration:
    CHANNELS = {'base': 15, 'shoulder': 14, 'elbow': 13, 'wrist-rotation': 12,
                'wrist-bend': 2, 'gripper-link': 1, 'gripper': 0}
    DURATION = 0.4
    COOLDOWN = 1.0
    HOLD_DURATION = 3.0
    SWEEP_RATE = 10.0
    SWEEP_WATCHDOG = 0.6

    def __init__(self, robot, clock=None, joint='base'):
        if joint not in self.CHANNELS:
            raise ValueError('Unsupported calibration joint')
        self.joint = joint
        self.channel = self.CHANNELS[joint]
        if joint == 'base':
            self.write = robot.calibration_base_write
        elif joint == 'shoulder':
            self.write = robot.calibration_shoulder_write
        else:
            self.write = lambda angle: robot.calibration_joint_write(joint, angle)
        self.robot = robot
        self.clock = clock or time.monotonic
        self.center = None
        self.last = None
        self.locked = False
        self.active = False
        self.fault = None
        self.deadline = 0
        self.ready_at = 0
        self.tokens = []
        self.mode = None
        self.active_token = None
        self.sweep_target = None
        self.sweep_sequence = 0
        self.sweep_seen = 0
        self.sweep_tick = 0

    def status(self):
        return {'protocol': 'miniarm-' + self.joint + '-pulse-v1', 'channel': self.channel,
                'duration_ms': 400, 'max_step': 1, 'window': 5,
                'center': self.center, 'last': self.last,
                'active': self.active, 'fault': self.fault,
                'locked': self.locked,
                'hold_protocol': 'miniarm-' + self.joint + '-hold-v1', 'hold_ms': 3000,
                'mode': self.mode, 'active_token': self.active_token,
                'sweep_protocol': 'miniarm-' + self.joint + '-sweep-v1',
                'sweep_ms': 3000, 'sweep_watchdog_ms': 600, 'sweep_rate': self.SWEEP_RATE,
                'sweep_target': self.sweep_target, 'sweep_sequence': self.sweep_sequence,
                'jog_protocol': 'miniarm-' + self.joint + '-jog-v1',
                'outputs': self.robot.calibration_outputs()}

    def begin(self, token, angle, expected, hold=False):
        if hold and (self.last is None or angle != self.last):
            raise ValueError('Run a pulse first; hold repeats only the last tested target')
        if not token or len(token) > 48 or any(c not in '0123456789abcdef-' for c in token):
            raise ValueError('Invalid pulse identifier')
        if token in self.tokens:
            raise ValueError('Duplicate pulse rejected; no automatic retries')
        if self.fault or self.active:
            raise ValueError('Calibration busy or faulted')
        if self.robot.simulate_hardware:
            raise ValueError('Physical controller required')
        if self.clock() < self.ready_at:
            raise ValueError('Wait one second between pulses')
        if isinstance(angle, bool) or not isinstance(angle, (int, float)) or not math.isfinite(angle) or not 0 <= angle <= 180:
            raise ValueError('Target must be a finite servo angle from 0 to 180')
        if expected != self.last:
            raise ValueError('Previous target changed; reconnect before testing')
        if self.last is not None and (abs(angle - self.last) > 1.000001 or abs(angle - self.center) > 5.000001):
            raise ValueError('Limit: one degree per change, five degrees from first target')
        if any(value is not None for value in self.robot.calibration_outputs()):
            raise ValueError('All seven outputs must already be off before a joint test')
        self.locked = True  # Raw pulses make the old model pose unusable for motion.
        self.tokens.append(token)
        self.tokens = self.tokens[-16:]
        self.active = True
        self.mode = 'hold' if hold else 'pulse'
        self.active_token = token
        self.deadline = self.clock() + (self.HOLD_DURATION if hold else self.DURATION)
        try:
            self.write(angle)
        except Exception:
            self.fault = 'Servo write failed. Disconnect servo power.'
            self.release()
            raise
        if self.center is None:
            self.center = angle
        self.last = angle

    def begin_hold(self, token, expected):
        self.begin(token, self.last, expected, hold=True)

    def release_hold(self, token):
        if self.active and (self.mode not in ('hold', 'sweep') or token != self.active_token):
            raise ValueError('Release does not own the active hold')
        self.release()

    def begin_sweep(self, token, expected):
        self.begin_hold(token, expected)
        self.mode = 'sweep'
        self.sweep_target = self.last
        self.sweep_sequence = 0
        self.sweep_seen = self.clock()
        self.sweep_tick = self.sweep_seen

    def update_sweep(self, token, sequence, target):
        # Expired sessions cannot be revived by a delayed packet.
        self.tick()
        if not self.active or self.mode != 'sweep' or self.active_token != token or self.fault:
            raise ValueError('Sweep inactive or request does not own it')
        if isinstance(sequence, bool) or not isinstance(sequence, int) or sequence != self.sweep_sequence + 1:
            raise ValueError('Stale or out-of-order sweep update')
        if isinstance(target, bool) or not isinstance(target, (int, float)) or not math.isfinite(target) or not 0 <= target <= 180 or abs(target - self.center) > 5.000001:
            raise ValueError('Sweep target outside the five-degree test window')
        self.sweep_sequence = sequence
        self.sweep_target = target
        self.sweep_seen = self.clock()

    def begin_jog(self, token, target, expected):
        if isinstance(target, bool) or not isinstance(target, (int, float)) or not math.isfinite(target) or not 0 <= target <= 180:
            raise ValueError('Invalid jog target')
        if self.center is not None and abs(target - self.center) > 5.000001:
            raise ValueError('Jog target outside the five-degree test window')
        if self.last is None:
            self.begin(token, target, expected)
            self.mode = 'sweep'
            self.deadline = self.clock() + self.HOLD_DURATION
            self.sweep_sequence = 0
            self.sweep_seen = self.clock()
            self.sweep_tick = self.sweep_seen
        else:
            self.begin_sweep(token, expected)
        self.sweep_target = target

    def keep_sweep(self, token, sequence):
        # Watchdog traffic carries no position and cannot change the target.
        self.tick()
        if not self.active or self.mode != 'sweep' or self.active_token != token or self.fault:
            raise ValueError('Sweep inactive or request does not own it')
        if isinstance(sequence, bool) or not isinstance(sequence, int) or sequence != self.sweep_sequence + 1:
            raise ValueError('Stale or out-of-order sweep keep-alive')
        self.sweep_sequence = sequence
        self.sweep_seen = self.clock()

    def tick(self):
        if not self.active:
            return
        now = self.clock()
        if now >= self.deadline or (self.mode == 'sweep' and now - self.sweep_seen >= self.SWEEP_WATCHDOG):
            self.release()
            return
        if self.mode == 'sweep':
            # Never catch up with a large jump after a stalled controller loop.
            step = self.SWEEP_RATE * min(max(0, now - self.sweep_tick), .05)
            self.sweep_tick = now
            change = max(-step, min(step, self.sweep_target - self.last))
            if abs(change) > .000001:
                target = self.last + change
                try:
                    self.write(target)
                    self.last = target  # Commanded angle, not encoder feedback.
                except Exception:
                    self.fault = 'Sweep write failed. Disconnect servo power.'
                    self.release()

    def release(self):
        if not self.active:
            return
        try:
            self.write(None)
        except Exception:
            self.fault = 'PWM release failed. Disconnect servo power.'
        finally:
            self.active = False
            self.mode = None
            self.active_token = None
            self.ready_at = self.clock() + self.COOLDOWN
