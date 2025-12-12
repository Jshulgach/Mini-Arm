"""
Circular Trajectory Demo

Execute a circular trajectory with the Mini-Arm end effector.
"""
import argparse
import time
import numpy as np
from mini_arm import MiniArmClient

def generate_circle_trajectory(center, radius, num_points=20, plane='xy'):
    """
    Generate points along a circular trajectory.
    
    Parameters:
    -----------
    center : list
        Center point [x, y, z]
    radius : float
        Circle radius in meters
    num_points : int
        Number of points along circle
    plane : str
        Plane for circle: 'xy', 'xz', or 'yz'
    
    Returns:
    --------
    trajectory : list of lists
        List of [x, y, z] points
    """
    theta = np.linspace(0, 2*np.pi, num_points)
    trajectory = []
    
    for t in theta:
        if plane == 'xy':
            x = center[0] + radius * np.cos(t)
            y = center[1] + radius * np.sin(t)
            z = center[2]
        elif plane == 'xz':
            x = center[0] + radius * np.cos(t)
            y = center[1]
            z = center[2] + radius * np.sin(t)
        elif plane == 'yz':
            x = center[0]
            y = center[1] + radius * np.cos(t)
            z = center[2] + radius * np.sin(t)
        
        trajectory.append([x, y, z])
    
    return trajectory

def main():
    parser = argparse.ArgumentParser(description='Execute circular trajectory')
    parser.add_argument('--port', type=str, default='COM3',
                       help='Serial port (default: COM3)')
    parser.add_argument('--center', type=float, nargs=3, default=[0.135, 0.0, 0.22],
                       help='Circle center [X Y Z] in meters (default: 0.135 0.0 0.22)')
    parser.add_argument('--radius', type=float, default=0.03,
                       help='Circle radius in meters (default: 0.03)')
    parser.add_argument('--points', type=int, default=20,
                       help='Number of points along circle (default: 20)')
    parser.add_argument('--plane', type=str, choices=['xy', 'xz', 'yz'], default='xy',
                       help='Plane for circle (default: xy)')
    parser.add_argument('--delay', type=float, default=0.5,
                       help='Delay between points in seconds (default: 0.5)')
    parser.add_argument('--verbose', action='store_true',
                       help='Enable verbose output')
    args = parser.parse_args()
    
    # Connect to Mini-Arm
    client = MiniArmClient(port=args.port, verbose=args.verbose)
    
    if not client.connected:
        print("❌ Failed to connect")
        return
    
    print(f"✅ Connected to Mini-Arm\n")
    
    # Generate trajectory
    print(f"🔄 Generating circular trajectory:")
    print(f"   Center: {args.center}")
    print(f"   Radius: {args.radius}m")
    print(f"   Points: {args.points}")
    print(f"   Plane: {args.plane}")
    
    trajectory = generate_circle_trajectory(args.center, args.radius, args.points, args.plane)
    
    # Execute trajectory
    print(f"\n▶️ Executing trajectory...")
    
    for i, point in enumerate(trajectory):
        print(f"Point {i+1}/{args.points}: {[f'{p:.3f}' for p in point]}")
        client.send(f'set_pose:{point}')
        time.sleep(args.delay)
    
    # Return to start
    print(f"\n🏁 Returning to start position...")
    client.send(f'set_pose:{trajectory[0]}')
    
    print("\n✅ Trajectory complete!")

if __name__ == '__main__':
    main()
