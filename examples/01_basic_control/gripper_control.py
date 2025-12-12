"""
Gripper Control Demo

Open and close the gripper, or set to a specific position.
"""
import argparse
import time
from mini_arm import MiniArmClient

def main():
    parser = argparse.ArgumentParser(description='Control Mini-Arm gripper')
    parser.add_argument('--port', type=str, default='COM3',
                       help='Serial port (default: COM3)')
    parser.add_argument('--action', type=str, choices=['open', 'close', 'cycle'],
                       default='cycle', help='Gripper action (default: cycle)')
    parser.add_argument('--position', type=float, default=None,
                       help='Set gripper to specific position (0-1)')
    parser.add_argument('--verbose', action='store_true',
                       help='Enable verbose output')
    args = parser.parse_args()
    
    client = MiniArmClient(port=args.port, verbose=args.verbose)
    
    if not client.connected:
        print("❌ Failed to connect")
        return
    
    print(f"✅ Connected to Mini-Arm\n")
    
    if args.position is not None:
        print(f"🤏 Setting gripper position to {args.position}")
        client.send(f'set_gripper:{args.position}')
    elif args.action == 'cycle':
        print("🔄 Cycling gripper: close → open → close")
        
        print("🤏 Closing...")
        client.send('set_gripper:close')
        time.sleep(2)
        
        print("🖐️ Opening...")
        client.send('set_gripper:open')
        time.sleep(2)
        
        print("🤏 Closing...")
        client.send('set_gripper:close')
    else:
        print(f"{'🤏 Closing' if args.action == 'close' else '🖐️ Opening'} gripper...")
        client.send(f'set_gripper:{args.action}')
    
    print("\n✅ Complete!")

if __name__ == '__main__':
    main()
