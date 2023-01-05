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
        self.create_node = self.create_service(
            CreateService, 'create_node', self.create_node_callback)
        self.delete_node = self.create_service(
            DeleteService, 'delete_node', self.delete_node_callback)
        self.get_scenes = self.create_service(
            SceneService, 'get_scenes', self.get_scenes_callback)

        self.record_bag = self.create_service(
            Empty, 'record_node', self.record_bag_callback)
        self.delete_record_node = self.create_service(
            DeleteService, 'delete_record_node', self.delete_record_node_callback)
        self.get_bag_name = self.create_service(
            SceneService, 'get_bag_name', self.get_bag_name_callback)

        self.create_driver_node = self.create_service(
            Empty, 'driver_node', self.create_driver_callback)
        self.delete_driver_node = self.create_service(
            DeleteService, 'delete_driver_node', self.delete_driver_node_callback)

        self.node_pub = self.create_publisher(String, 'node_list', 5)
        self.cp_pub = self.create_publisher(Int32, 'cp_value', 5)
        self.clear_pub = self.create_publisher(Int32, 'clear_value', 5)
        self.up_pub = self.create_publisher(Int32, 'up_value', 5)
        self.eject_pub = self.create_publisher(Int32, 'eject_value', 5)
        timer_period = 1.0
        self.tmr = self.create_timer(timer_period, self.timer_callback)

        # process pid of launch scene
        self.scene_pid = -1
        self.scene_subprocess = None

        # process pid of record bag
        self.record_bag_pid = -1
        self.subprocess = None
        self.bag_name = ''

        # process pid of launch driver
        self.driver_node_pid = -1
        self.driver_node_subprocess = None
        cmd_test = ['ls']
        self.pro_cp = subprocess.Popen(cmd_test, shell=True,stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        self.pro_clear = subprocess.Popen(cmd_test, shell=True,stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        self.pro_up = subprocess.Popen(cmd_test, shell=True,stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        self.pro_eject = subprocess.Popen(cmd_test, shell=True,stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        # the folder path of each scenes
        self.scene_path = os.path.join(os.path.join(
            os.path.expanduser('~')), 'Desktop/data')
        self.cp_state = 200
        self.clear_state = 2
        self.up_state = 2
        self.eject_state = 2
        self.cp_start = False
        self.clear_start = False
        self.up_start = False
        self.eject_start = False
        self.cp_node = self.create_service(
            Empty, 'cp_node', self.cp_node_callback)
        self.clear_node = self.create_service(
            Empty, 'clear_node', self.clear_node_callback)
        self.up_node = self.create_service(
            Empty, 'up_node', self.up_node_callback)
        self.eject_node = self.create_service(
            Empty, 'eject_node', self.eject_node_callback)
        self.v0 = 0
        self.target = 0
        print('Copy start!')
    def cp_node_callback(self, request, response):

        # self.cp_state = 1
        print('Copy start!')
        cmd2 = ['ls /dev/ | grep "sd"']
        pro_cp_= subprocess.Popen(cmd2,shell=True,stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        SDISK = []
        print('111111')
        SDISK = pro_cp_.stdout.readlines()
        for i in SDISK:
            if ("sda1" in str(i[:-1])):
                cmd2 = ['sudo  mount /dev/sda1 /mnt/usb']
                pro_cp_ = subprocess.Popen(cmd2,stdout = subprocess.PIPE,stderr = subprocess.PIPE,shell=True)
                print('222222')
                break
            if ("sdb1" in str(i[:-1])):
                cmd2 = ['sudo  mount /dev/sdb1 /mnt/usb']
                pro_cp_ = subprocess.Popen(cmd2,stdout = subprocess.PIPE,stderr = subprocess.PIPE,shell=True)
                break
            if ("sdc1" in str(i[:-1])):
                cmd2 = ['sudo  mount /dev/sdc1 /mnt/usb']
                pro_cp_ = subprocess.Popen(cmd2,stdout = subprocess.PIPE,stderr = subprocess.PIPE,shell=True)
                break
            if ("sdd1" in str(i[:-1])):
                cmd2 = ['sudo  mount /dev/sdd1 /mnt/usb']
                pro_cp_ = subprocess.Popen(cmd2,stdout = subprocess.PIPE,stderr = subprocess.PIPE,shell=True)
                break
        while(pro_cp_.poll()  is  None):{}
        cmd2 = ['sudo chmod -R 777 /home/lukas/Desktop/bag/']
        pro_cp_ = subprocess.Popen(cmd2,shell=True,stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        # while(pro_cp_.poll()  is  None):{}

        while(pro_cp_.poll()  is  None):{}
        cmd2 = ['du -sh  -b  /mnt/usb/bag']
        pro_cp_ = subprocess.Popen(cmd2,shell=True,stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        s = pro_cp_.stdout.readlines()[0]   
        self.v0 = int(s.decode("utf-8").split('/mnt/usb/bag')[0])
        print('self.v0')
        print(self.v0)
        while(pro_cp_.poll()  is  None):{}
        cmd2 = ['du -sh  -b  /home/lukas/Desktop/bag']
        pro_cp_ = subprocess.Popen(cmd2,shell=True,stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        s = pro_cp_.stdout.readlines()[0]   
        self.target = int(s.decode("utf-8").split('/home/lukas/Desktop/bag')[0])
        print('self.target')
        print(self.target)
        # cmd2 = ["sudo cp", "-r", "/home/lukas/Desktop/bag/*", "/mnt/usb/bag"]
        cmd2 = ['sudo cp -r /home/lukas/Desktop/bag/* /mnt/usb/bag']
        self.pro_cp = subprocess.Popen(cmd2,shell=True,stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        # self.pro_cp = subprocess.Popen(cmd2,shell=False,stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        # print(self.pro_cp.stderr.readlines())
        # file_ = open('/home/lukas/Desktop/1.txt', mode='wb+')
        # file_ = open('/home/lukas/Desktop/3.txt', mode='wb+')
        # file_.write(self.pro_cp.stderr.readline())
        # file_.write(self.pro_cp.stdout.readline())
        # cmd2 = ['sudo mkdir /media/dzt/TU100Pro/bag/test2.txt']
        # self.pro_cp = subprocess.Popen(cmd2,shell=True,stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        self.cp_start = True
        return response

    def clear_node_callback(self, request, response):

        self.clear_state = 1
        print('Clear start!')
        cmd2 = ['rm -rf /home/lukas/Desktop/bag/*']
        self.pro_clear = subprocess.Popen(cmd2,shell=True)
        self.clear_start = True
        return response
    def up_node_callback(self, request, response):

        self.up_state = 1
        print('Up start!')
        cmd2 = ['ls /dev/ | grep "sd"']
        self.pro_up = subprocess.Popen(cmd2,shell=True,stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        SDISK = []
        SDISK = self.pro_up.stdout.readlines() 
        for i in SDISK:
            if ("sda1" in str(i[:-1])):
                cmd2 = ['sudo  mount /dev/sda1 /mnt/usb']
                self.pro_up = subprocess.Popen(cmd2,stdout = subprocess.PIPE,stderr = subprocess.PIPE,shell=True)
                break
            if ("sdb1" in str(i[:-1])):
                cmd2 = ['sudo  mount /dev/sdb1 /mnt/usb']
                self.pro_up = subprocess.Popen(cmd2,stdout = subprocess.PIPE,stderr = subprocess.PIPE,shell=True)
                break
            if ("sdc1" in str(i[:-1])):
                cmd2 = ['sudo  mount /dev/sdc1 /mnt/usb']
                self.pro_up = subprocess.Popen(cmd2,stdout = subprocess.PIPE,stderr = subprocess.PIPE,shell=True)
                break
            if ("sdd1" in str(i[:-1])):
                cmd2 = ['sudo  mount /dev/sdd1 /mnt/usb']
                self.pro_up = subprocess.Popen(cmd2,stdout = subprocess.PIPE,stderr = subprocess.PIPE,shell=True)
                break
        while(self.pro_up.poll()  is  None):{}
        cmd2 = ['sudo cp -r /mnt/usb/data/* /home/lukas/Desktop/data']
        self.pro_up = subprocess.Popen(cmd2,shell=True,stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        # file_ = open('/home/lukas/Desktop/err.txt', mode='wb+')
        # file_.write(self.pro_cp.stderr.readline())
        # file_ = open('/home/lukas/Desktop/out.txt', mode='wb+')
        # file_.write(self.pro_cp.stdout.readline())
        self.up_start = True
        return response
    def eject_node_callback(self, request, response):

        self.eject_state = 1
        print('Eject start!')
        cmd2 = ['ls /dev/ | grep "sd"']
        pro_eject_ = subprocess.Popen(cmd2,shell=True,stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        EDISK = []
        while(pro_eject_.poll()  is  None):{}
        EDISK = pro_eject_.stdout.readlines() 
        for i in EDISK:
            if ("sda1" in str(i[:-1])):
                cmd2 = ['sudo  eject /dev/sda1']
                self.pro_eject = subprocess.Popen(cmd2,stdout = subprocess.PIPE,stderr = subprocess.PIPE,shell=True)
                break
            if ("sdb1" in str(i[:-1])):
                cmd2 = ['sudo  eject /dev/sdb1']
                self.pro_eject = subprocess.Popen(cmd2,stdout = subprocess.PIPE,stderr = subprocess.PIPE,shell=True)
                break
            if ("sdc1" in str(i[:-1])):
                cmd2 = ['sudo  mount /dev/sdc1']
                self.pro_eject = subprocess.Popen(cmd2,stdout = subprocess.PIPE,stderr = subprocess.PIPE,shell=True)
                break
            if ("sdd1" in str(i[:-1])):
                cmd2 = ['sudo  mount /dev/sdd1']
                self.pro_eject = subprocess.Popen(cmd2,stdout = subprocess.PIPE,stderr = subprocess.PIPE,shell=True)
                break
        self.eject_start = True
        return response

    def create_node_callback(self, request, response):
        # when you already open one pid, so pid != -1
        if self.scene_pid != -1 or request.scene == '':
            response.result = False
            logging.error(
                'you have already open a launch, could not open another one !!!!!!!!!!!!')
            return response

        # process of command : tracking_backend.launch.xml
        cmd = ['ros2', 'launch', 'dt_launch', 'tracking_backend.launch.xml',
               'scene_path:=' + self.scene_path + '/' + request.scene]
        self.scene_subprocess = subprocess.Popen(cmd)
        self.scene_pid = self.scene_subprocess.pid
        response.result = True
        return response

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
        --qos-profile-overrides-path /home/lukas/dtsimulation.ros2/record_qos.yaml
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

    def create_driver_callback(self, request, response):
        # when you already open one pid, so pid != -1
        if self.driver_node_pid != -1:
            logging.error(
                'you have already open driver node !!!!!!!!!!!!')

        # process of command : tracking_backend.launch.xml
        cmd = ['ros2', 'launch', 'dt_launch', 'driver.launch.xml']
        self.driver_node_subprocess = subprocess.Popen(cmd)
        self.driver_node_pid = self.driver_node_subprocess.pid
        logging.info('start driver', self.driver_node_pid,
                     '!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! ')
        return response

    def delete_node_callback(self, _, response):

        # when you get default pid
        if self.scene_pid == -1:
            response.result = False
            logging.error('no pid to kill !!!!!!!!!!!!')
            return response

        # kill process of ros2 launch
        cmd = "kill -2 " + str(self.scene_pid)
        os.system(cmd)
        response.result = True
        logging.info('Already kill pid : ', self.scene_pid,
                     " !!!!!!!!!!!!!!!!!!!!!!")
        self.scene_subprocess, self.scene_pid = None, -1
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

    def delete_driver_node_callback(self, _, response):

        # when you get default pid
        if self.driver_node_pid == -1:
            response.result = False
            logging.error('no pid to kill !!!!!!!!!!!!')
            return response

        # kill process of ros2 launch driver
        cmd = "kill -2 " + str(self.driver_node_pid)
        os.system(cmd)
        response.result = True
        logging.info('going to kill record bag: ', self.driver_node_pid,
                     " !!!!!!!!!!!!!!!!!!!!!!")
        self.driver_node_subprocess, self.driver_node_pid = None, -1

        return response

    def get_scenes_callback(self, _, response):

        scene_folder_list = self.get_only_dirs_list(self.scene_path)
        response.scenes = scene_folder_list
        print('get all scene folders !!!!!!!!!! Here are these folder:',
              scene_folder_list)
        return response

    def get_bag_name_callback(self, _, response):

        response.scenes = [self.bag_name]
        print('get all scene folders !!!!!!!!!! Here are these folder:',
              response.scenes)
        return response
    def timer_callback(self):
        msg = String()
        msg.data = str(self.get_node_namespace_dict())
        self.node_pub.publish(msg)

        msg2 = Int32()
        print('self.cp_state0 ')
        print(self.cp_state )
        if(self.cp_start == True):
            print('self.cp_state1 ')
            print(self.cp_state )
            if(self.pro_cp.poll()  is  None):
                cmd_request = ['du -sh  -b  /mnt/usb/bag']
                pro_request_ = subprocess.Popen(cmd_request,shell=True,stdout=subprocess.PIPE, stderr=subprocess.PIPE)
                s = pro_request_.stdout.readlines()[0]   
                v1 = int(s.decode("utf-8").split('/mnt/usb/bag')[0])
                print(v1 - self.v0)
                print(self.target)
                self.cp_state = int(100 * (v1 - self.v0) / self.target)
                # self.cp_state = 1
                print('self.cp_state2 ')
                print(self.cp_state )
                # self.cp_state = 1
            else:
                self.cp_state = self.pro_cp.poll()
                print('self.cp_state3 ')
                print(self.cp_state )
        msg2.data = self.cp_state
        self.cp_pub.publish(msg2)

        msg3 = Int32()
        if(self.clear_start == True):
            if(self.pro_clear.poll()  is  None):
                self.clear_state = 1
            else:
                self.clear_state = self.pro_clear.poll()
        msg3.data = self.clear_state
        self.clear_pub.publish(msg3)

        msg4 = Int32()
        if(self.up_start == True):
            if(self.pro_up.poll()  is  None):
                self.up_state = 1
            else:
                self.up_state = self.pro_up.poll()
        msg4.data = self.up_state
        self.up_pub.publish(msg4)

        msg5 = Int32()
        if(self.eject_start == True):
            if(self.pro_eject.poll()  is  None):
                self.eject_state = 1
            else:
                self.eject_state = self.pro_eject.poll()
        msg5.data = self.eject_state
        self.eject_pub.publish(msg5)

    def get_only_dirs_list(self, path):
        try:
            out = []
            dir_list = os.listdir(path)
            for x in dir_list:
                if os.path.isdir(path+'/'+x):
                    out.append(x)
            return out
        except FileNotFoundError:
            print('get invalid value of scens path ')
            return []

    def get_node_namespace_dict(self):
        node_ns_list = self.get_node_names_and_namespaces()
        node_ns_dict = {}
        # list of default node when open backend_control and bridge_suit
        back_list = ['backend_control', '_ros2cli_daemon_0',
                     'rosapi', 'rosbridge_websocket', 'rosapi_params']
        for x in node_ns_list:
            node_ns = x[1].split('/')[1]  # get name of name_space

            # other name space contain name has no name space
            if node_ns == '' and x[0] in back_list:
                node_ns = 'backend'
            if node_ns == '' and x[0].find('_ros2cli_daemon') >= 0:
                node_ns = 'backend'
            elif node_ns == '':
                node_ns = 'other'

            # generate dictionary of node
            if node_ns_dict.__contains__(node_ns):
                node_ns_dict[node_ns] += 1
            else:
                node_ns_dict[node_ns] = 1

        return json.dumps(node_ns_dict)



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
