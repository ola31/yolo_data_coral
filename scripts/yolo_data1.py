#!/usr/bin/env python
# -*- coding: utf8 -*- 

from socket import*
import time


import rospy
from turtlesim.msg import Pose
from darknet_ros_msgs.msg import BoundingBoxes
from darknet_ros_msgs.msg import BoundingBox
from std_msgs.msg import String

ip = "127.0.0.1"
port = 12347

def print_turtle_pos():
    rospy.init_node('print_turtle_pos', anonymous=False)
    rospy.Subscriber("/turtle1/pose", Pose, callback)
    #yolo_pub=rospy.Publisher("/darknet_ros/bounding_boxes",BoundingBoxes, queue_size = 1000)
    chatt_pub=rospy.Publisher("/chatt",String, queue_size = 1000)

    yolo_data=BoundingBoxes()
    chatt_msg=String()
    #rospy.spin()
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        clientSocket.send("I am a client".encode("UTF-8"))
        print("send_massage.")
        data = clientSocket.recv(1024)  #데이터 수신
        #print("recieved data :"+data.decode("UTF-8"))
        data2=data.decode("UTF-8")

        x1_s = data2.split("|")[0]
        y1_s = data2.split("|")[1]
        x2_s = data2.split("|")[2]
        y2_s = data2.split("|")[3]

        x1 = int(x1_s)
        y1 = int(y1_s)
        x2 = int(x2_s)
        y2 = int(y2_s)

        print(x1)
        print(y1)
        print(x2)
        print(y2)

        
        

    	'''yolo_data.header.stamp=rospy.Time.now()
    	yolo_data.header.frame_id ="detection"
    	yolo_data.image_header.frame_id ="detection"
    	yolo_data.bounding_boxes[10].probability = 0.7
    	yolo_data.bounding_boxes[0].xmin = 301
    	yolo_data.bounding_boxes[0].xmax = 171
    	yolo_data.bounding_boxes[0].ymin = 555
    	yolo_data.bounding_boxes[0].ymax = 480
    	yolo_data.bounding_boxes[0].id = 0
    	yolo_data.bounding_boxes[0].Class = "person"

    	yolo_pub.publish(yolo_data)'''
        chatt_msg.data=data2
        chatt_pub.publish(chatt_msg)
    	rate.sleep()
    clientSocket.close()  # 연결 종료 



    #rospy.spin()

def callback(data):
    rospy.loginfo("Turtle Pose X : %s, Y : %s", data.x, data.y)

if __name__ == '__main__':

    clientSocket = socket(AF_INET,SOCK_STREAM)  #소켓 생성
    clientSocket.connect((ip,port))   # 서버와 연결

    print("connected")
    print_turtle_pos()
