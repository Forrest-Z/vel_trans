import os
import subprocess
import logging
import rclpy
import json
from datetime import datetime
from rclpy.node import Node
from backend_control_msgs.srv import CreateService, SceneService, DeleteService
from std_srvs.srv._empty import Empty
from std_msgs.msg import String
from std_msgs.msg import Int32

class BackendControl(Node):

    def __init__(self):
        super().__init__('backend_control')
        # create node and delete node service
 

        self.record_bag = self.create_service(
            Empty, 'record_node', self.record_bag_callback)
        self.delete_record_node = self.create_service(
            DeleteService, 'delete_record_node', self.delete_record_node_callback)
        self.get_bag_name = self.create_service(
            SceneService, 'get_bag_name', self.get_bag_name_callback)



        # process pid of record bag
        self.record_bag_pid = -1
        self.subprocess = None
        self.bag_name = ''



    

    def record_bag_callback(self, request, response):
        # when you already open one pid, so pid != -1
        if self.record_bag_pid != -1:
            logging.error(
                'you have already open record a bag !!!!!!!!!!!!')
            return response


        timenow = datetime.now()
        str_datetime = timenow.strftime("%Y_%m_%d_%H_%M_%S")
        # process of record bag
        cmd_string = """
        ros2 bag record /driver/imu /driver/steer /odom /localization/twist 
        /vehicle/status/steering /vehicle/status/control_mode  
        /rslidar_points /tf /tf_static /map/vector_map_marker 
        /diagnostics_err /sensing/lidar/no_ground/pointcloud 
        /perception/object_recognition/tracking/objects/visualization 
        /planning/scenario_planning/lane_driving/motion_planning/surround_obstacle_checker/debug/marker 
        /planning/scenario_planning/lane_driving/motion_planning/obstacle_stop_planner/debug/marker 
        /control/trajectory_follower/predicted_trajectory 
        /control/trajectory_follower/debug/marker/marker_array 
        /planning/scenario_planning/lane_driving/motion_planning/obstacle_avoidance_planner/trajectory  
        --qos-profile-overrides-path /home/lukas/.product/dtsimulation.ros2/record_qos.yaml
        """
        logging.info(cmd_string)
        cmd_string = cmd_string.replace('\n', '')
        cmd = list(cmd_string.split(" "))
        new_cmd = list(set(cmd))
        new_cmd.sort(key=cmd.index)
        new_cmd.remove('')
        new_cmd+=['-o']+[os.path.join(os.path.join(
            os.path.expanduser('~')), 'Desktop/bag/') + "rosbag"+str_datetime]

        self.bag_name = "rosbag"+str_datetime
        print(new_cmd)
        self.record_bag_subprocess = subprocess.Popen(new_cmd)
        self.record_bag_pid = self.record_bag_subprocess.pid
        logging.info('start to record bag ', self.record_bag_pid,
                     '!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! ')
        return response

    def delete_record_node_callback(self, _, response):

        # when you get default pid
        if self.record_bag_pid == -1:
            response.result = False
            logging.error('no pid to kill !!!!!!!!!!!!')
            return response

        # kill process of ros2 launch
        cmd = "kill -2 " + str(self.record_bag_pid)
        os.system(cmd)
        response.result = True
        logging.info('going to kill record bag: ', self.record_bag_pid,
                     " !!!!!!!!!!!!!!!!!!!!!!")
        self.record_bag_subprocess, self.record_bag_pid = None, -1

        self.bag_name = ""
        return response


    def get_bag_name_callback(self, _, response):

        response.scenes = [self.bag_name]
        print('get all scene folders !!!!!!!!!! Here are these folder:',
              response.scenes)
        return response






def main(args=None):

    rclpy.init(args=args)
    backend_control = BackendControl()
    rclpy.spin(backend_control)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    backend_control.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
