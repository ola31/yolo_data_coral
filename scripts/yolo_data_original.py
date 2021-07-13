#!/usr/bin/env python
import rospy
from turtlesim.msg import Pose
from darknet_ros_msgs.msg import BoundingBoxes
from darknet_ros_msgs.msg import BoundingBox

def print_turtle_pos():
    rospy.init_node('print_turtle_pos', anonymous=False)
    rospy.Subscriber("/turtle1/pose", Pose, callback)
    yolo_pub=rospy.Publisher("/darknet_ros/bounding_boxes",BoundingBoxes, queue_size = 1000)

    yolo_data=BoundingBoxes()
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
    	yolo_data.header.stamp=rospy.Time.now()
    	yolo_data.header.frame_id ="detection"
    	yolo_data.image_header.frame_id ="detection"
    	yolo_data.bounding_boxes[10].probability = 0.7
    	yolo_data.bounding_boxes[0].xmin = 301
    	yolo_data.bounding_boxes[0].xmax = 171
    	yolo_data.bounding_boxes[0].ymin = 555
    	yolo_data.bounding_boxes[0].ymax = 480
    	yolo_data.bounding_boxes[0].id = 0
    	yolo_data.bounding_boxes[0].Class = "person"

    	yolo_pub.publish(yolo_data)
    	rate.Sleep()



    rospy.spin()

def callback(data):
    rospy.loginfo("Turtle Pose X : %s, Y : %s", data.x, data.y)

if __name__ == '__main__':
    print_turtle_pos()