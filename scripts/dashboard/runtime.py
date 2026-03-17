#!/usr/bin/env python3
import argparse
import time

try:
    from dashboard.app import ROS_IMPORTS_AVAILABLE, SwarmDashboard, rospy
except ImportError:
    from .app import ROS_IMPORTS_AVAILABLE, SwarmDashboard, rospy


def main():
    parser = argparse.ArgumentParser(description="Nexus Swarm web dashboard")
    parser.add_argument("--demo", action="store_true", help="Run with synthetic demo data instead of ROS topics")
    parser.add_argument("--host", default=None, help="HTTP bind host")
    parser.add_argument("--port", type=int, default=None, help="HTTP bind port")
    parser.add_argument("--drone-prefix", default=None, help="Vehicle prefix for demo mode or ROS override")
    args, _ = parser.parse_known_args()

    auto_demo = not ROS_IMPORTS_AVAILABLE
    demo_mode = args.demo or auto_demo

    if demo_mode:
        if auto_demo and not args.demo:
            print("[SwarmDashboard] ROS modules not found. Starting in demo mode.", flush=True)
        dashboard = SwarmDashboard(
            demo_mode=True,
            host=args.host,
            port=args.port,
            drone_prefix=args.drone_prefix,
        )
        try:
            while True:
                time.sleep(1.0)
        except KeyboardInterrupt:
            dashboard._shutdown()
        return

    rospy.init_node("swarm_dashboard", anonymous=False)
    SwarmDashboard(
        demo_mode=False,
        host=args.host,
        port=args.port,
        drone_prefix=args.drone_prefix,
    )
    rospy.spin()
