import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from threading import Thread 
from pymoveit2 import MoveIt2

coordinate = [0. , 0. ,0. ,0. ,0. ,0. ]
moveit_working_coordinate_1=[-3.379206669941175, 7.570075322437978e-05, 2.8038473220306458e-05, 4.8531018645157585e-06, -9.046280425926246e-05, -1.1254802047516442e-05]
moveit_working_coordinate_2=[1.066539274851356, -1.0242831851043602e-05, 3.0097671158461052e-05, 7.052046605246998e-05, 2.083876999134197e-05, -3.8500817007421746e-05]
moveit_working_coordinate_3=[-0.21837540468916455, -5.953373336657528e-05, -3.319308154426898e-05, -9.246559585202746e-05, -8.091495978514677e-06, 7.018560373342842e-05]
wrist_rotate_1=[-0.1500555889561294, -5.922298603666299e-05, 9.88231176434864e-05, 6.970593139905323e-05, -7.351189874911744e-05, -1.6678609569908764]
wrist_rotate_2=[-0.14999295564410395, 8.602873195472766e-05, -3.3890736706115426e-05, 4.567809475566884e-05, -7.642541918842819e-05, -3.952641508815276]
startup_ur_position=[-0.15, 2.655422146752303e-11, 4.997734752432139e-11, 1.3690325140897081e-11, 4.267112828444705e-19, 4.774224402013394e-11]

def main():
    rclpy.init()
    node=rclpy.create_node(node_name="order_target_location")
    logger = node.get_logger()

    callback_ur = ReentrantCallbackGroup()

    moveit2_callback_ur = MoveIt2(node=node,
                      joint_names=['ur5_shoulder_pan_joint','ur5_shoulder_lift_joint',
                                   'ur5_elbow_joint','ur5_wrist_1_joint',
                                   'ur5_wrist_2_joint', 'ur5_wrist_3_joint'],
                      base_link_name='ur5_base_link',
                      end_effector_name='Gripper',
                      group_name='Arm',
                      callback_group=callback_ur                 )
    
    execute_it=rclpy.executors.MultiThreadedExecutor(2)
    execute_it.add_node(node)
    executor_thread_target=Thread(target=execute_it.spin, daemon=True, args=())
    executor_thread_target.start()
    node.create_rate(1.0).sleep()

    logger.info("Move To Target Position")

    moveit2_callback_ur.move_to_configuration(moveit_working_coordinate_1)
    moveit2_callback_ur.wait_until_executed()

    rclpy.shutdown()
    executor_thread_target.join()

if __name__=='__main__':
    main()