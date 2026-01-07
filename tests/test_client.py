"""Unit tests for mini_arm package."""

import pytest
from unittest.mock import Mock, patch, MagicMock


class TestMiniArmImports:
    """Test that package imports work correctly."""
    
    def test_import_package(self):
        """Test importing the main package."""
        import mini_arm
        assert hasattr(mini_arm, 'MiniArmClient')
        assert hasattr(mini_arm, '__version__')
    
    def test_import_client(self):
        """Test importing MiniArmClient directly."""
        from mini_arm import MiniArmClient
        assert MiniArmClient is not None
    
    def test_version_format(self):
        """Test version string format."""
        from mini_arm import __version__
        parts = __version__.split('.')
        assert len(parts) >= 2
        assert all(p.isdigit() for p in parts[:2])


class TestMiniArmClient:
    """Test MiniArmClient class without hardware."""
    
    @patch('mini_arm.client.serial.Serial')
    def test_client_creation(self, mock_serial):
        """Test creating a client with mocked serial."""
        mock_serial_instance = MagicMock()
        mock_serial_instance.is_open = True
        mock_serial_instance.in_waiting = 0
        mock_serial.return_value = mock_serial_instance
        
        from mini_arm import MiniArmClient
        client = MiniArmClient(port='COM3')
        
        assert client.connected is True
        assert client.name == 'MiniArmClient'
    
    @patch('mini_arm.client.serial.Serial')
    def test_client_send_message(self, mock_serial):
        """Test sending a message."""
        mock_serial_instance = MagicMock()
        mock_serial_instance.is_open = True
        mock_serial_instance.in_waiting = 0
        mock_serial.return_value = mock_serial_instance
        
        from mini_arm import MiniArmClient
        client = MiniArmClient(port='COM3')
        
        client.send('test_command')
        mock_serial_instance.write.assert_called()
    
    @patch('mini_arm.client.serial.Serial')
    def test_client_context_manager(self, mock_serial):
        """Test using client as context manager."""
        mock_serial_instance = MagicMock()
        mock_serial_instance.is_open = True
        mock_serial_instance.in_waiting = 0
        mock_serial.return_value = mock_serial_instance
        
        from mini_arm import MiniArmClient
        
        with MiniArmClient(port='COM3') as client:
            assert client.connected is True
        
        mock_serial_instance.close.assert_called()
    
    @patch('mini_arm.client.serial.Serial')
    def test_home_command(self, mock_serial):
        """Test home command."""
        mock_serial_instance = MagicMock()
        mock_serial_instance.is_open = True
        mock_serial_instance.in_waiting = 0
        mock_serial.return_value = mock_serial_instance
        
        from mini_arm import MiniArmClient
        client = MiniArmClient(port='COM3')
        
        client.home()
        
        # Check that 'home' was sent
        calls = mock_serial_instance.write.call_args_list
        assert any(b'home' in call[0][0] for call in calls)


class TestMotionCommands:
    """Test motion-related methods."""
    
    @patch('mini_arm.client.serial.Serial')
    def test_set_pose(self, mock_serial):
        """Test set_pose method."""
        mock_serial_instance = MagicMock()
        mock_serial_instance.is_open = True
        mock_serial_instance.in_waiting = 0
        mock_serial.return_value = mock_serial_instance
        
        from mini_arm import MiniArmClient
        client = MiniArmClient(port='COM3')
        
        client.set_pose(0.135, 0.0, 0.22)
        
        calls = mock_serial_instance.write.call_args_list
        assert any(b'set_pose' in call[0][0] for call in calls)
    
    @patch('mini_arm.client.serial.Serial')
    def test_set_delta_pose(self, mock_serial):
        """Test set_delta_pose method."""
        mock_serial_instance = MagicMock()
        mock_serial_instance.is_open = True
        mock_serial_instance.in_waiting = 0
        mock_serial.return_value = mock_serial_instance
        
        from mini_arm import MiniArmClient
        client = MiniArmClient(port='COM3')
        
        client.set_delta_pose(dx=0.01, dy=0.0, dz=-0.01)
        
        calls = mock_serial_instance.write.call_args_list
        assert any(b'set_delta_pose' in call[0][0] for call in calls)
    
    @patch('mini_arm.client.serial.Serial')
    def test_set_joints(self, mock_serial):
        """Test set_joints method."""
        mock_serial_instance = MagicMock()
        mock_serial_instance.is_open = True
        mock_serial_instance.in_waiting = 0
        mock_serial.return_value = mock_serial_instance
        
        from mini_arm import MiniArmClient
        client = MiniArmClient(port='COM3')
        
        client.set_joints([0, 45, 90, 0, 45, 0])
        
        calls = mock_serial_instance.write.call_args_list
        assert any(b'movemotors' in call[0][0] for call in calls)


class TestGripperCommands:
    """Test gripper methods."""
    
    @patch('mini_arm.client.serial.Serial')
    def test_set_gripper(self, mock_serial):
        """Test set_gripper method."""
        mock_serial_instance = MagicMock()
        mock_serial_instance.is_open = True
        mock_serial_instance.in_waiting = 0
        mock_serial.return_value = mock_serial_instance
        
        from mini_arm import MiniArmClient
        client = MiniArmClient(port='COM3')
        
        client.set_gripper(90)
        
        calls = mock_serial_instance.write.call_args_list
        assert any(b'set_gripper' in call[0][0] for call in calls)
    
    @patch('mini_arm.client.serial.Serial')
    def test_gripper_open_close(self, mock_serial):
        """Test gripper_open and gripper_close methods."""
        mock_serial_instance = MagicMock()
        mock_serial_instance.is_open = True
        mock_serial_instance.in_waiting = 0
        mock_serial.return_value = mock_serial_instance
        
        from mini_arm import MiniArmClient
        client = MiniArmClient(port='COM3')
        
        client.gripper_open()
        client.gripper_close()
        
        calls = mock_serial_instance.write.call_args_list
        assert any(b'open' in call[0][0] for call in calls)
        assert any(b'close' in call[0][0] for call in calls)


class TestLEDCommands:
    """Test LED methods."""
    
    @patch('mini_arm.client.serial.Serial')
    def test_set_led(self, mock_serial):
        """Test set_led method."""
        mock_serial_instance = MagicMock()
        mock_serial_instance.is_open = True
        mock_serial_instance.in_waiting = 0
        mock_serial.return_value = mock_serial_instance
        
        from mini_arm import MiniArmClient
        client = MiniArmClient(port='COM3')
        
        client.set_led(255, 0, 0)
        
        calls = mock_serial_instance.write.call_args_list
        assert any(b'set_led' in call[0][0] for call in calls)
    
    @patch('mini_arm.client.serial.Serial')
    def test_led_colors(self, mock_serial):
        """Test LED color convenience methods."""
        mock_serial_instance = MagicMock()
        mock_serial_instance.is_open = True
        mock_serial_instance.in_waiting = 0
        mock_serial.return_value = mock_serial_instance
        
        from mini_arm import MiniArmClient
        client = MiniArmClient(port='COM3')
        
        client.led_red()
        client.led_green()
        client.led_blue()
        client.led_off()
        
        # Should have 4 set_led calls
        calls = [c for c in mock_serial_instance.write.call_args_list if b'set_led' in c[0][0]]
        assert len(calls) == 4


class TestTrajectoryCommands:
    """Test trajectory methods."""
    
    @patch('mini_arm.client.serial.Serial')
    def test_start_trajectory(self, mock_serial):
        """Test start_trajectory method."""
        mock_serial_instance = MagicMock()
        mock_serial_instance.is_open = True
        mock_serial_instance.in_waiting = 0
        mock_serial.return_value = mock_serial_instance
        
        from mini_arm import MiniArmClient
        client = MiniArmClient(port='COM3')
        
        client.start_trajectory("circle", repeat=True)
        
        calls = mock_serial_instance.write.call_args_list
        assert any(b'trajectory' in call[0][0] for call in calls)
    
    @patch('mini_arm.client.serial.Serial')
    def test_stop_trajectory(self, mock_serial):
        """Test stop_trajectory method."""
        mock_serial_instance = MagicMock()
        mock_serial_instance.is_open = True
        mock_serial_instance.in_waiting = 0
        mock_serial.return_value = mock_serial_instance
        
        from mini_arm import MiniArmClient
        client = MiniArmClient(port='COM3')
        
        client.stop_trajectory()
        
        calls = mock_serial_instance.write.call_args_list
        assert any(b'stop' in call[0][0] for call in calls)


class TestSystemCommands:
    """Test system commands."""
    
    @patch('mini_arm.client.serial.Serial')
    def test_set_debug(self, mock_serial):
        """Test set_debug method."""
        mock_serial_instance = MagicMock()
        mock_serial_instance.is_open = True
        mock_serial_instance.in_waiting = 0
        mock_serial.return_value = mock_serial_instance
        
        from mini_arm import MiniArmClient
        client = MiniArmClient(port='COM3')
        
        client.set_debug(True)
        client.set_debug(False)
        
        calls = mock_serial_instance.write.call_args_list
        assert any(b'debug:on' in call[0][0] for call in calls)
        assert any(b'debug:off' in call[0][0] for call in calls)
    
    @patch('mini_arm.client.serial.Serial')
    def test_set_rate(self, mock_serial):
        """Test set_rate method."""
        mock_serial_instance = MagicMock()
        mock_serial_instance.is_open = True
        mock_serial_instance.in_waiting = 0
        mock_serial.return_value = mock_serial_instance
        
        from mini_arm import MiniArmClient
        client = MiniArmClient(port='COM3')
        
        client.set_rate(100)
        
        calls = mock_serial_instance.write.call_args_list
        assert any(b'set_rate:100' in call[0][0] for call in calls)


if __name__ == '__main__':
    pytest.main([__file__, '-v'])

