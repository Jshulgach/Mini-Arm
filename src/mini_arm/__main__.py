"""
Command-line interface for Mini-Arm control.

Usage:
    python -m mini_arm --port COM3
    python -m mini_arm --port COM3 --command "get_pose"
    python -m mini_arm --port COM3 --interactive
"""

import sys
import time
import argparse

from .client import MiniArmClient


def main():
    """Command-line interface for Mini-Arm control."""
    parser = argparse.ArgumentParser(
        description='Mini-Arm Robot Control Interface',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Connect and get help
  python -m mini_arm --port COM3
  
  # Send a command
  python -m mini_arm --port COM3 --command "get_pose"
  
  # Move to position
  python -m mini_arm --port COM3 --command "set_pose:[0.135,0.0,0.22]"
  
  # Interactive mode
  python -m mini_arm --port COM3 --interactive

For more examples, see the examples/ directory.
        """
    )
    
    parser.add_argument(
        '--port', type=str, default='COM3',
        help='Serial port (default: COM3, Linux: /dev/ttyACM0)'
    )
    parser.add_argument(
        '--baudrate', type=int, default=9600,
        help='Baudrate for serial connection (default: 9600)'
    )
    parser.add_argument(
        '--command', type=str, default='help',
        help='Command to send to Mini-Arm (default: help)'
    )
    parser.add_argument(
        '--interactive', action='store_true',
        help='Enter interactive mode for sending multiple commands'
    )
    parser.add_argument(
        '--verbose', action='store_true',
        help='Enable verbose output'
    )
    
    args = parser.parse_args()
    
    # Connect to Mini-Arm
    print(f"Connecting to Mini-Arm on {args.port}...")
    client = MiniArmClient(
        port=args.port,
        baudrate=args.baudrate,
        verbose=args.verbose
    )
    
    if not client.connected:
        print("❌ Failed to connect to Mini-Arm")
        print("   Check port name and ensure device is connected")
        return 1
    
    print("✅ Connected successfully!\n")
    
    if args.interactive:
        print("Interactive mode - type 'exit' or 'quit' to exit")
        print("Type 'help' to see available commands\n")
        
        while True:
            try:
                cmd = input("mini-arm> ").strip()
                if cmd.lower() in ['exit', 'quit', 'q']:
                    break
                if cmd:
                    client.send(cmd)
                    time.sleep(0.1)
                    response = client.get_buffer()
                    if response:
                        print(response)
            except KeyboardInterrupt:
                print("\n")
                break
            except EOFError:
                break
    else:
        # Send single command
        client.send(args.command)
        time.sleep(0.5)
        response = client.get_buffer()
        if response:
            print(response)
    
    print("\n✅ Disconnecting...")
    client.disconnect()
    return 0


if __name__ == '__main__':
    sys.exit(main())
