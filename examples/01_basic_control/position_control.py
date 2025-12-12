"""
Cartesian Position Control Demo

Move the Mini-Arm's end effector to specific XYZ coordinates.
"""
import argparse
import time
from mini_arm import MiniArmClient

def main():
    parser = argparse.ArgumentParser(description='Control Mini-Arm end effector position')
    parser.add_argument('--port', type=str, default='COM3',
                       help='Serial port (default: COM3)')
    parser.add_argument('--x', type=float, default=0.135,
                       help='Target X position in meters (default: 0.135)')
    parser.add_argument('--y', type=float, default=0.0,
                       help='Target Y position in meters (default: 0.0)')
    parser.add_argument('--z', type=float, default=0.22,
                       help='Target Z position in meters (default: 0.22)')
    parser.add_argument('--verbose', action='store_true',
                       help='Enable verbose output')
    args = parser.parse_args()
    
    # Connect to Mini-Arm
    client = MiniArmClient(port=args.port, verbose=args.verbose)
    
    if not client.connected:
        print("❌ Failed to connect")
        return
    
    print(f"✅ Connected to Mini-Arm\n")
    
    # Move to home first
    print("🏠 Moving to home position...")
    client.send('home')
    time.sleep(2)
    
    # Move to target position
    target = [args.x, args.y, args.z]
    print(f"📍 Moving to position: {target}")
    client.send(f'set_pose:{target}')
    time.sleep(2)
    
    # Read current position
    print("\n📊 Current position:")
    client.send('get_pose')
    
    print("\n✅ Movement complete!")

if __name__ == '__main__':
    main()
