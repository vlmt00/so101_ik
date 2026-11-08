"""Command line interface for the SO-101 debug simulator."""
import argparse
from .model import RobotModel, TCPConfig
from .kinematics import Kinematics
from .collision import CollisionChecker
from .controller import SimulationController

def main(argv=None):
    parser = argparse.ArgumentParser(description="SO-101 constrained IK/FK simulator")
    parser.add_argument("--urdf", required=True)
    parser.add_argument("--tcp-xyz", nargs=3, type=float, required=True, metavar=("X","Y","Z"),
                        help="Measured moving-jaw-link to TCP offset in metres")
    parser.add_argument("--tcp-rpy", nargs=3, type=float, default=(0,0,0))
    parser.add_argument("--tip-axis", nargs=3, type=float, default=(0,0,1))
    parser.add_argument("--target", nargs=3, type=float, metavar=("X","Y","Z"))
    parser.add_argument("--viewer", action="store_true")
    parser.add_argument("--allow-unchecked-collision", action="store_true",
                        help="Debug only: override unavailable collision checking")
    args = parser.parse_args(argv)
    if not args.viewer and args.target is None:
        parser.error("Specify --viewer or --target X Y Z")
    tcp = TCPConfig(xyz=tuple(args.tcp_xyz), rpy=tuple(args.tcp_rpy), tip_axis=tuple(args.tip_axis))
    with RobotModel(args.urdf, tcp) as model:
        kin = Kinematics(model)
        ctrl = SimulationController(model, kin, CollisionChecker(model),
                                    allow_unchecked_collision=args.allow_unchecked_collision)
        print("joint names:", model.joint_names)
        print("initial TCP XYZ [m]:", kin.fk(ctrl.q)[:3,3].tolist())
        if args.target is not None:
            print(ctrl.move_tcp(args.target))
        if args.viewer:
            from .viewer import SwiftViewer
            SwiftViewer(model, ctrl).run()

if __name__ == "__main__":
    main()
