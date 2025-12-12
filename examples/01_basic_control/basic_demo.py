"""
Basic Mini-Arm Control Demo

Demonstrates how to connect to the Mini-Arm and send simple commands.
"""
import argparse
from mini_arm import MiniArmClient

def main():
    parser = argparse.ArgumentParser(description='Basic Mini-Arm control demo')
    parser.add_argument('--port', type=str, default='COM3', 
                       help='Serial port (default: COM3, Linux: /dev/ttyACM0)')
    parser.add_argument('--baudrate', type=int, default=9600,
                       help='Baudrate for serial connection (default: 9600)')
    parser.add_argument('--verbose', action='store_true',
                       help='Enable verbose output')
    args = parser.parse_args()
    
    # Connect to Mini-Arm
    print(f"Connecting to Mini-Arm on {args.port}...")
    client = MiniArmClient(port=args.port, baudrate=args.baudrate, verbose=args.verbose)
    
    if not client.connected:
        print("❌ Failed to connect to Mini-Arm")
        return
    
    print("✅ Connected successfully!\n")
    
    # Get available commands
    print("Getting available commands...")
    client.send('help')
    
    # Get current robot state
    print("\n📍 Getting current pose...")
    client.send('get_pose')
    
    print("\n🔧 Getting joint states...")
    client.send('get_joints')
    
    # Move to home position
    print("\n🏠 Moving to home position...")
    client.send('home')
    
    # Set LED color
    print("\n💡 Setting LED to green...")
    client.send('set_led:[0,255,0]')
    
    print("\n✅ Demo complete!")

if __name__ == '__main__':
    main()
