#!/usr/bin/env python

import rospy
import cv2
import apriltag
from april_detection.msg import AprilTagDetectionArray
from april_detection.msg import Pose
# from std_msgs.msg import Float64MultiArray
import numpy as np
import time
import tf

# pose_pub = rospy.Publisher('/current_pose', Float64MultiArray, queue_size=1)
pose_pub = rospy.Publisher('/current_pose', Pose, queue_size=1)


def tag_callback(msg):
    pose_msg = Pose()
    pose_msg.header.stamp = msg.header.stamp
    for detection in msg.detections:
        
        apriltag_id = detection.id
        position = detection.pose.position
        position = np.array([[position.x], [position.y], [position.z]])
        tag_orientation = detection.pose.orientation 

        r = tf.transformations.quaternion_matrix([tag_orientation.x, tag_orientation.y, 
        tag_orientation.z, tag_orientation.w])[:3,:3]

        cTa = np.append(np.append(r, position,axis=1), [[0,0,0,1]], axis=0)

        rTa = np.matmul(rTc, cTa)
        print("AprilTag in robot coordinates rTa: \n",rTa)
        aTr = np.linalg.inv(rTa)
        wTa = pose_ma[apriltag_id]
        wTr = np.matmul(wTa,aTr)
        
        print("Robot in world coordinates wTr: \n",wTr)
        pose_msg.pose = list(wTr.flatten())
        pose_pub.publish(pose_msg)


if __name__ == "__main__":
    rospy.init_node('localization_node')
    rospy.Subscriber("/apriltag_detection_array", AprilTagDetectionArray, tag_callback)
    rospy.spin()