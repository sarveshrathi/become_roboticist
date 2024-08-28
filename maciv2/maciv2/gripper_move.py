import rclpy
from rclpy.node import Node
from rclpy.action.client import ActionClient
from control_msgs.action import FollowJointTrajectory
from control_msgs.msg import JointTolerance
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
import argparse

def main(args=None):
    rclpy.init(args=args)
    node1 =Node("gripper_control")
    logger = node1.get_logger()
    logger.info("Node launched. Ok Bye")
    parser = argparse.ArgumentParser()
    parser.add_argument("target_position", type=float)
    args=parser.parse_args()
    #logger.info(print(args.target_position))

    #target_position=-0.05
    logger.info("target position set")
    action_variable=ActionClient(node1, FollowJointTrajectory,"/Gripper_controller/follow_joint_trajectory")
    logger.info("action variable set")
    action_variable.wait_for_server()

    logger.info("Server connection achieved")

    trajectory_joint=JointTrajectory()
    trajectory_joint.joint_names=["finger_joint"]
    trajectory_joint.points=[JointTrajectoryPoint(positions=[args.target_position])]

    target=FollowJointTrajectory.Goal()
    target.trajectory=trajectory_joint
    target.goal_time_tolerance=Duration(sec=1)

    tolerance_joint=[JointTolerance(name=n, position=0.001, velocity=0.001) for n in trajectory_joint.joint_names]

    target.path_tolerance=tolerance_joint
    target.goal_tolerance=tolerance_joint

    action_goal=action_variable.send_goal_async(target)
    logger.info("Witness trajectory completion Hurrayyyy ")

    rclpy.spin_until_future_complete(node1,action_goal)

if __name__=='__main__':
    main()