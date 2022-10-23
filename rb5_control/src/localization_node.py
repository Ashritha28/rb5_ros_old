#!/usr/bin/env python

import rospy
import cv2
import apriltag
from april_detection.msg import AprilTagDetectionArray, AprilTagDetection
# from navigation_dev.msg import Pose 
import numpy as np
import time
import tf
from scipy.spatial.transform import Rotation

#pose_pub = rospy.Publisher('/current_pose', Pose, queue_size=1)

# Location of the marker AprilTag
pose_ma = {8: np.asarray([[0, -1, 0, 2.05],[0, 0, -1, 0.015], [1, 0, 0, 0.15], [0,0,0,1]])}

# Camera in robot frame
rTc = np.asarray([[0, -1, 0, 0.05], [0, 0, -1, 0.015], [0,0,1, 0.15], [0,0,0,1]])

def tag_callback(msg):

    for detection in msg.detections:

        #print("Type:",type(detection))
        apriltag_id = detection.id
        position = np.array(detection.pose.position)
        print("ID:", apriltag_id)
        print("Position:", position)
        tag_orientation = detection.pose.orientation 
        print("Tag oreintation quaternion:", tag_orientation)
        
        r = np.array(Rotation.from_quat(tag_orientation))
        print("Rotation matrix using scipy:", r)
        print("Rotation Matrix using tf: \n", tf.transformations.quaternion_matrix([tag_orientation.x, tag_orientation.y, 
        tag_orientation.z, tag_orientation.w]))
        cTa = (np.concatenate((np.concantenate((r, position), axis=1),np.array([0,0,0,1])),axis=0)
        print("cTa: \n",cTa)
        aTc = np.linalg.inv(cTa)
        # AprilTag in robot coordinates
        rTa = np.matmul(aTc, rTc)
        print("AprilTag in robot coordinates rTa: \n",rTa)
        # Robot in world coordinates
        wTr = np.matmul(pose_ma[apriltag_id], rTa)
        print("Robot in world coordinates wTr: \n",wTr)
        # new[apriltag_id] = wTr
        # pose_msg.pose.matrix = list(wTr[:3, :].flatten())

    
        # print("ID:",msg.id,"\n Pose:",msg.pose)
        # for id, pose in zip(msg.id, msg.pose) :
            # print("april_tag detetcion:", detection)


if __name__ == "__main__":
    rospy.init_node('localization_node')
    rospy.Subscriber("/apriltag_detection_array", AprilTagDetectionArray, tag_callback)
    rospy.spin()