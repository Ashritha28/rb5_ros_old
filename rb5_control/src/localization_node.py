#!/usr/bin/env python

import rospy
import cv2
import apriltag
from april_detection.msg import AprilTagDetectionArray, AprilTagDetection
# from navigation_dev.msg import Pose 
import numpy as np
import time

#pose_pub = rospy.Publisher('/current_pose', Pose, queue_size=1)

# Location of the marker AprilTag
pose_ma = {4: np.asarray([[1, 0, 0, 0],[0, 1, 0, 0], [0,0,1, 1.7], [0,0,0,1]]),
            0: np.asarray([[1, 0, 0, 0],[0, 1, 0, 0], [0,0,1, 2.8], [0,0,0,1]]),
            12: np.asarray([[1, 0, 0, 0],[0, 1, 0, 0], [0,0,1, 2.8], [0,0,0,1]])}

# Camera in robot frame
rTc = np.asarray([[-1, 0, 0, 0], [0, -1, 0, 0], [0,0,1, -0.10], [0,0,0,1]])

def tag_callback(msg):

    for detection in msg.detections:
        print("Type:",type(detection))
        print("ID:", detection.id)
        print("Pose:", detection.pose)
        # print("ID:",msg.id,"\n Pose:",msg.pose)
        # for id, pose in zip(msg.id, msg.pose) :
            # print("april_tag detetcion:", detection)

    # pose_msg = Pose()
    # pose_msg.header.stamp = msg.header.stamp
    # robot_r = rospy.Rate(5)
    # if len(msg.detections):
    #     new = {}
    #     for apriltag_id, pose in zip(msg.ids, msg.detections):
    #             # Location of AprilTag in camera coordinates
    #             cTa = np.asarray(list(pose.matrix)).reshape((4,4))
    #             # Inverse of cTa - Location of camera with respect to april tag
    #             aTc = np.linalg.inv(cTa)
    #             # AprilTag in robot coordinates
    #             rTa = np.matmul(aTc, rTc)
    #             # Robot in world coordinates
    #             wTr = np.matmul(pose_ma[4], rTa)
    #             new[apriltag_id] = wTr
    #             pose_msg.pose.matrix = list(wTr[:3, :].flatten())
    #             pose_pub.publish(pose_msg)
    # robot_r.sleep()

# def tag_callback1(msg):
#     if len(msg.)



if __name__ == "__main__":
    rospy.init_node('localization_node')
    rospy.Subscriber("/apriltag_detection_array", AprilTagDetectionArray, tag_callback)
    # rospy.
    # rospy.Subscriber("/april_poses",AprilTagDetection, tag_callback )
    rospy.spin()