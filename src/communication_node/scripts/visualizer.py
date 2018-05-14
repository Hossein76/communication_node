#!/usr/bin/env python

import rospy;
from visualization_msgs.msg import Marker,MarkerArray;
from nav_msgs.msg import Odometry;
from geometry_msgs.msg import Point;


class OdomWrapper:
    def __init__(self,robot_name_space):
        self.robot_name_space=robot_name_space;
        self.odom_subscriber=rospy.Subscriber("/"+robot_name_space+"/odom", Odometry, self.odom_callback);
        self.robot_x=None;
        self.robot_y=None;
    def odom_callback(self,odom_data):
        self.robot_x=odom_data.pose.pose.position.x;
        self.robot_y=odom_data.pose.pose.position.y;



def main():
    rospy.init_node("visualizer_node");
    Odoms_list=[];
    rospy.sleep(2.0);
    rate=rospy.Rate(1.0);
    connection_visualization_publisher=rospy.Publisher("/connection_graph", MarkerArray, queue_size=100);
    robots_list=rospy.get_param("/robots_list");
    for i in robots_list:
        Odoms_list.append(OdomWrapper(i));
    rospy.sleep(2.0);
    while not rospy.is_shutdown():
        test_tempx=0;
        test_tempy=0;
        test_connection_markers=[];
        id_counter=0;
        for j in range(0,len(robots_list)):
            connection_list=(rospy.get_param("/direct_connection_list_"+robots_list[j]));
            for i in range (j+2,len(connection_list)):
                test_tempx=None;
                test_tempx=Odoms_list[i-1].robot_x;
                test_tempy=Odoms_list[i-1].robot_y;
                if(test_tempx==None):continue;
                test_marker = Marker();
                test_marker.header.frame_id = "/map";
                test_marker.header.stamp = rospy.Time.now();
                test_marker.ns = "points_and_lines";
                test_marker.id = id_counter;
                id_counter+=1;
                test_marker.type = Marker.LINE_STRIP;
                test_marker.action = Marker.ADD;
                test_marker.pose.orientation.x = 0.0;
                test_marker.pose.orientation.y = 0.0;
                test_marker.pose.orientation.z = 0.0;
                test_marker.pose.orientation.w = 1.0;
                test_marker.scale.x = 0.1;
                test_marker.color.r = 1-int(connection_list[i]);
                test_marker.color.g = int(connection_list[i]);
                test_marker.color.b = 0.5;
                test_marker.color.a = 1.0;
                test_marker.points.append(Point(Odoms_list[j].robot_x,Odoms_list[j].robot_y,0.0));
                test_marker.points.append(Point(test_tempx,test_tempy,0.0));
                test_marker.lifetime = rospy.Duration(10.0);
                test_connection_markers.append(test_marker);
        connection_visualization_publisher.publish(test_connection_markers);
        rate.sleep();


    rospy.spin();

if __name__ == '__main__':
    main();
