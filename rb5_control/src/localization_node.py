#!/usr/bin/env python

import rospy
import cv2
import apriltag
from april_detection.msg import AprilTagDetectionArray
from std_msgs.msg import Float64MultiArray
# from navigation_dev.msg import Pose 
import numpy as np
import time
import tf
#from scipy.spatial.transform import Rotation

pose_pub = rospy.Publisher('/current_pose', Float64MultiArray, queue_size=1)

# Location of the marker AprilTag
pose_ma = {8: np.asarray([[0, -1, 0, 2.05],[0, 0, -1, 0.015], [1, 0, 0, 0.15], [0,0,0,1]])}

# Camera in robot frame
rTc = np.asarray([[0, -1, 0, 0.05], [0, 0, -1, 0.015], [1,0,0, 0.15], [0,0,0,1]])

def tag_callback(msg):

    for detection in msg.detections:

        #print("Type:",type(detection))
        apriltag_id = detection.id
        position = detection.pose.position
        position = np.array([[position.x], [position.y], [position.z]])
        print("ID:", apriltag_id)
        print("Position:", position)
        tag_orientation = detection.pose.orientation 
        print("Tag oreintation quaternion:", tag_orientation)
        
        # r = np.array(Rotation.from_quat(tag_orientation))
        # print("Rotation matrix using scipy:", r)
        r = tf.transformations.quaternion_matrix([tag_orientation.x, tag_orientation.y, 
        tag_orientation.z, tag_orientation.w])[:3,:3]
        print("Rotation Matrix using tf: \n", r)
        print("Size of rotation matrix:", r.shape)
        print("Size of position matrix:",position.shape)
        cTa = np.append(np.append(r, position,axis=1), [[0,0,0,1]], axis=0)
        print("cTa: \n",cTa)
        aTc = np.linalg.inv(cTa)
        cTr = np.linalg.inv(rTc)
        rTa = np.matmul(cTr, cTa)
        print("AprilTag in robot coordinates rTa: \n",rTa)
        # AprilTag in robot coordinates
        # rTa = np.matmul(aTc, rTc) -- v
        # rTa = np.matmul(rTc, cTa) -- first attempt
        # aTr = np.matmul(aTc, rTc) -- second attempt
        
        # aTr = np.matmul(cTa, cTr)
        # print("AprilTag in robot coordinates rTa: \n",rTa) --v
        # print("AprilTag in robot coordinates rTa: \n",aTr)
        # Robot in world coordinates
        # rTa = np.linalg.inv(aTr)
        wTa = pose_ma[apriltag_id]
        aTw = np.linalg.inv(wTa)
        aTr = np.linalg.inv(rTa)
        wTr = np.matmul(aTw,aTr)
        # wTr = np.matmul(pose_ma[apriltag_id], aTr)
        print("Robot in world coordinates wTr: \n",wTr)
        # new[apriltag_id] = wTr
        pose_pub.publish(wTr)

if __name__ == "__main__":
    rospy.init_node('localization_node')
    rospy.Subscriber("/apriltag_detection_array", AprilTagDetectionArray, tag_callback)
    rospy.spin()