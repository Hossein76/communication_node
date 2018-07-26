#!/usr/bin/env python
# coding=utf-8
"""Environment Information.

# Authors:  MohammadHossein GohariNejad <hoseingohari76@gmail.com>
# License:  BSD 3 clause

"""

import os
import signal
import sys
from time import gmtime,strftime
import rospy;
import tf;
from geometry_msgs.msg import *;
from nav_msgs.msg import *;
from std_msgs.msg import *;
import threading;
import math

robots_list=["robot0","robot1","robot2","robot3"];
info_list=[];
map_logger=None;
path_logger=None;
base_time=0;

def on_exit(*args):
    global information_logger
    print ( "\n EXITING MESSAGE HANDLER")
    if path_logger!=None :
         path_logger.write("\n The Test has finished on "+strftime("%Y-%m-%d %H:%M:%S", gmtime()) + " GMT time \n")
         path_logger.write("\n ======================== \n ======================== \n \n \n")
         path_logger.close()
    sys.exit(0)

class path_wrapper:
    def __init__(self,robot_name_space,t_lock_path,subscribing_topic):
        self.path_sub=rospy.Subscriber("/"+robot_name_space+"/"+subscribing_topic, Path, self.path_callback);
        self.poses=[];
        self.poses_lenght=0;
        self.path_lenght=0.0;
        self.t_lock_path=t_lock_path;
        self.robot=robot_name_space;

    def path_callback(self,input_path_data):
        self.t_lock_path.acquire();
        self.poses=list(input_path_data.poses);
        self.t_lock_path.release();

    def path_lenght_calculator(self):
        self.t_lock_path.acquire();
        for i in range(self.poses_lenght,len(self.poses)):
            if (i==0):continue
            x=(self.poses[i].pose.position.x-self.poses[i-1].pose.position.x)*(self.poses[i].pose.position.x-self.poses[i-1].pose.position.x)
            y=(self.poses[i].pose.position.y-self.poses[i-1].pose.position.y)*(self.poses[i].pose.position.y-self.poses[i-1].pose.position.y)
            self.path_lenght+=float("{0:.2f}".format(math.sqrt(y+x)));
        self.poses_lenght=len(self.poses);
        self.t_lock_path.release();
        return self.path_lenght;




def main():
    global info_list;
    global path_logger;
    global map_logger;
    global base_time;
    global logfile;
    global robots_list;
    signal.signal(signal.SIGINT, on_exit)
    signal.signal(signal.SIGTERM, on_exit)
    rospy.init_node('path_node', anonymous=True)
    debuger_mode=True;
    if debuger_mode==True:
        log_file=rospy.get_param("log_file",default="results")
        log_folder=rospy.get_param("log_folder",default="map")
        if not os.path.exists("/home/sosvr/communication_node_project/communication_node2/results_pack/"+log_folder):
            os.makedirs("/home/sosvr/communication_node_project/communication_node2/results_pack/"+log_folder)
        path_logger =  open("/home/sosvr/communication_node_project/communication_node2/results_pack/"+log_folder+"/"+log_file+"_path.log", "w")
        path_logger.write("\n \n \n ###################### \n ###################### \n")
        path_logger.write("\n This is the result of test on "+strftime("%Y-%m-%d %H:%M:%S", gmtime()) + " GMT time \n")

    for i in robots_list:
        info_list.append(path_wrapper(i,threading.Lock(),"trajectory"));

    rate = rospy.Rate(0.05)
    base_time = 0;
    while (not rospy.is_shutdown()) and base_time<1500:
        if(base_time%100==0):print(str(base_time/100),"seconds");
        if debuger_mode==True:
            for i in info_list:
                i.path_lenght_calculator();
                path_logger.write("\n "+i.robot+","+str(i.path_lenght)+" ,"+str(int(base_time)));
        base_time+=20;
        rate.sleep();
    print("finished");
    rospy.spin()

main();
