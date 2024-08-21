import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from threading import Thread 
from pymoveit2 import MoveIt2

pick_waypoint_1=[-0.7768169702938305, 0.9595365580391205, 0.8453268765405033, -8.793195753264711e-05, -0.7311592546153743, 1.2794062800786332, -1.376837213138566e-06]
pick_waypoint_2=[-0.7768937823999034, 0.959587177545769, 0.8453333183178307, 2.079577089334219e-05, -8.663337421849671e-05, 1.5536354286967344, -1.6473118644116209e-06]
pick_waypoint_3=[-0.776769991209331, 0.9595148317567568, 0.8454367349226957, -7.212510327687826e-06, 0.21837196699511388, 1.2795208990029185, -0.013553927619205037]
pick_waypoint_4=[-0.8226082206904352, 0.9596730405484704, 0.845417702534286, 1.9762709252006413e-06, 0.36539394230531386, 1.3708809129042396, -0.023593655889561235]
pick_waypoint_5=[-5.16353874435608, 0.9595218494483508, 0.845361256539499, 5.4320167081640266e-05, 0.36557330149872863, 1.3709450454069159, -0.023783837315884186]
pick_waypoint_6=[-5.163538744355872, 0.9595218494487204, 0.8453612565380715, 5.4320159512771706e-05, 0.36557330160710755, 1.3709450454497922, -0.002208954982193562]
pick_waypoint_7=[-5.163550065260846, 0.959541369680499, 0.8454650009054442, -8.022191312363221e-05, -0.7997586509624279, 1.3937548944767948, -0.002208954982274742]
pick_waypoint_8=[2.479801452714567e-05, 4.051984490753366e-05, -3.500173391349459e-05, 1.9706518869629192e-05, 4.2031898164317405e-06, 9.027974184260614e-05, -0.002208954982133421]

simple_pick_1=[-7.075690636411287e-05, 0.6853523751538583, 0.8338684837784254, -2.5593085854721977e-05, -0.7082308606376221, 1.576502690510742, 3.508010022209804e-11]
simple_pick_2=[-7.760298857465411e-05, 0.6854776768903642, 0.8340249780158362, 3.812050365586654e-05, 0.34270297777897724, 1.5764124204015484, 5.550576466360751e-11]
simple_pick_3=[]
simple_pick_4=[]
simple_pick_5=[]
simple_pick_6=[]
simple_pick_7=[]
simple_pick_8=[]
simple_pick_9=[]
simple_pick_10=[]


def main():
    rclpy.init()
    node=rclpy.create_node(node_name="pick_and_place")
    logger = node.get_logger()

    callback_ur = ReentrantCallbackGroup()

    moveit2_callback_ur = MoveIt2(node=node,
                      joint_names=['ur5_shoulder_pan_joint','ur5_shoulder_lift_joint',
                                   'ur5_elbow_joint','ur5_wrist_1_joint',
                                   'ur5_wrist_2_joint', 'ur5_wrist_3_joint','finger_joint'],#'left_inner_finger_joint', 'right_outer_knuckle_joint', 'right_inner_finger_joint'],
                      base_link_name='ur5_base_link',
                      end_effector_name='Gripper',
                      group_name='Arm',
                      callback_group=callback_ur                 )
    
    execute_it=rclpy.executors.MultiThreadedExecutor(2)
    execute_it.add_node(node)
    executor_thread_target=Thread(target=execute_it.spin, daemon=True, args=())
    executor_thread_target.start()
    node.create_rate(1.0).sleep()

    logger.info("Pick and place started")

    moveit2_callback_ur.move_to_configuration(simple_pick_1)
    moveit2_callback_ur.wait_until_executed()

    moveit2_callback_ur.move_to_configuration(simple_pick_2)
    moveit2_callback_ur.wait_until_executed()

    '''moveit2_callback_ur.move_to_configuration(pick_waypoint_3)
    moveit2_callback_ur.wait_until_executed()

    moveit2_callback_ur.move_to_configuration(pick_waypoint_4)
    moveit2_callback_ur.wait_until_executed()

    moveit2_callback_ur.move_to_configuration(pick_waypoint_5)
    moveit2_callback_ur.wait_until_executed()

    moveit2_callback_ur.move_to_configuration(pick_waypoint_6)
    moveit2_callback_ur.wait_until_executed()

    moveit2_callback_ur.move_to_configuration(pick_waypoint_7)
    moveit2_callback_ur.wait_until_executed()

    moveit2_callback_ur.move_to_configuration(pick_waypoint_8)
    moveit2_callback_ur.wait_until_executed()'''



    rclpy.shutdown()
    executor_thread_target.join()

if __name__=='__main__':
    main()