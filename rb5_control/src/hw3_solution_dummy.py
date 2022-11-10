#!/usr/bin/env python
import sys
import roslib
import rospy
import geometry_msgs.msg
from geometry_msgs.msg import Twist
import numpy as np
import math
import tf
import tf2_ros
from tf.transformations import quaternion_matrix

"""
The class of the pid controller.
"""
class PIDcontroller:
    def __init__(self, Kp, Ki, Kd):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.target = None
        self.I = np.array([0.0,0.0,0.0])
        self.lastError = np.array([0.0,0.0,0.0])
        self.timestep = 0.1
        self.maximumValue = 0.02

    def setTarget(self, targetx, targety, targetw):
        """
        set the target pose.
        """
        self.I = np.array([0.0,0.0,0.0]) 
        self.lastError = np.array([0.0,0.0,0.0])
        self.target = np.array([targetx, targety, targetw])

    def setTarget(self, state):
        """
        set the target pose.
        """
        self.I = np.array([0.0,0.0,0.0]) 
        self.lastError = np.array([0.0,0.0,0.0])
        self.target = np.array(state)

    def getError(self, currentState, targetState):
        """
        return the different between two states
        """
        result = targetState - currentState
        result[2] = (result[2] + np.pi) % (2 * np.pi) - np.pi
        return result 

    def setMaximumUpdate(self, mv):
        """
        set maximum velocity for stability.
        """
        self.maximumValue = mv

    def update(self, currentState):
        """
        calculate the update value on the state based on the error between current state and target state with PID.
        """
        e = self.getError(currentState, self.target)

        P = self.Kp * e
        self.I = self.I + self.Ki * e * self.timestep 
        I = self.I
        D = self.Kd * (e - self.lastError)
        result = P + I + D

        self.lastError = e

        # scale down the twist if its norm is more than the maximum value. 
        resultNorm = np.linalg.norm(result)
        if(resultNorm > self.maximumValue):
            result = (result / resultNorm) * self.maximumValue
            self.I = 0.0

        return result

def isRotationMatrix(R) :
    Rt = np.transpose(R)
    shouldBeIdentity = np.dot(Rt, R)
    I = np.identity(3, dtype = R.dtype)
    n = np.linalg.norm(I - shouldBeIdentity)
    return n < 1e-6

def rotationMatrixToEulerAngles(R) :
 
    assert(isRotationMatrix(R))
 
    sy = math.sqrt(R[0,0] * R[0,0] +  R[1,0] * R[1,0])
 
    singular = sy < 1e-6
 
    if not singular :
        x = math.atan2(R[2,1] , R[2,2])
        y = math.atan2(-R[2,0], sy)
        z = math.atan2(R[1,0], R[0,0])
    else :
        x = math.atan2(-R[1,2], R[1,1])
        y = math.atan2(-R[2,0], sy)
        z = 0
    #print(np.array([x, y, z]))
    return np.array([x, y, z])

def getCurrentPos(l):
    """
    Given the tf listener, we consider the camera's z-axis is the header of the car
    """
    br = tf.TransformBroadcaster()
    result = None
    foundSolution = False
    # print("HELLO")

    for i in range(0, 9):
        camera_name = "camera_" + str(i)
        if l.frameExists(camera_name):
            # print("Frame Exists")
            try:
                now = rospy.Time()
                # print("TRYYY")
                # wait for the transform ready from the map to the camera for 1 second.
                print(l.getFrameStrings())
                print(camera_name)
                # detections = filter(lambda i: 'camera_' in i, l.getFrameStrings())
                # print(detections)
                # l.waitForTransform("map", camera_name, now, rospy.Duration(2))
                l.waitForTransform(camera_name, "marker_"+str(i) , now, rospy.Duration(1))
                # print("Transform available")
                # extract the transform camera pose in the map coordinate.
                # (trans, rot) = l.lookupTransform("map", camera_name, now)
                (trans, rot) = l.lookupTransform(camera_name, "marker_"+str(i) , now)
                # convert the rotate matrix to theta angle in 2d
                # print(trans, rot)
                matrix = quaternion_matrix(rot)
                # angle = math.atan2(matrix[1][2], matrix[0][2])
                eulerangles = rotationMatrixToEulerAngles(matrix[0:3,0:3])
                angle = eulerangles[2]
                print(matrix.shape, trans.shape)
                aTc = np.append(np.append(matrix[0:3,0:3], trans,axis=1), [[0,0,0,1]], axis=0)
                cTa = np.linalg.inv(aTc)
                print(cTa)
                print("Euler Angles:", eulerangles)
                print("Matrix, angle:",matrix, angle)
                # this is not required, I just used this for debug in RVIZ
                # br.sendTransform((trans[0], trans[1], 0), tf.transformations.quaternion_from_euler(0,0,angle), rospy.Time.now(), "base_link", "map")
                result = np.array([trans[2], trans[0], angle])
                foundSolution = True
                # break
            except (tf.LookupException):
                print("Lookup error")
            except(tf.ConnectivityException):
                print("Connectivity Error")
            except(tf.ExtrapolationException, tf2_ros.TransformException):
                print("Extrapolation and Transform Exception")
    listener.clear()
    return foundSolution, result


def genTwistMsg(desired_twist):
    """
    Convert the twist to twist msg.
    """
    twist_msg = Twist()
    twist_msg.linear.x = desired_twist[0] 
    twist_msg.linear.y = desired_twist[1] 
    twist_msg.linear.z = 0
    twist_msg.angular.x = 0
    twist_msg.angular.y = 0
    twist_msg.angular.z = desired_twist[2]
    return twist_msg

def coord(twist, current_state):
    """
    Convert the twist into the car coordinate
    """
    J = np.array([[np.cos(current_state[2]), np.sin(current_state[2]), 0.0],
                  [-np.sin(current_state[2]), np.cos(current_state[2]), 0.0],
                  [0.0,0.0,1.0]])
    return np.dot(J, twist)
    


if __name__ == "__main__":
    import time
    rospy.init_node("hw3")
    pub_twist = rospy.Publisher("/twist", Twist, queue_size=1)

    listener = tf.TransformListener()

    waypoint = np.array([[0.0,0.0,0.0], 
                         [1.0,0.0,0.0],
                         [1.0,2.0,np.pi],
                         [0.0,0.0,0.0]])

    # init pid controller
    pid = PIDcontroller(0.1,0.005,0.005)

    # init current state
    current_state = np.array([0.0,0.0,0.0])

    # in this loop we will go through each way point.
    # once error between the current state and the current way point is small enough, 
    # the current way point will be updated with a new point.
    for wp in waypoint:
        print("move to way point", wp)
        # set wp as the target point
        pid.setTarget(wp)

        # calculate the current twist
        update_value = pid.update(current_state)
        # publish the twist
        pub_twist.publish(genTwistMsg(coord(update_value, current_state)))
        #print(coord(update_value, current_state))
        time.sleep(0.05)
        # update the current state
        current_state += update_value
        found_state, estimated_state = getCurrentPos(listener)
        print(found_state, estimated_state)
        if found_state: # if the tag is detected, we can use it to update current state.
            current_state = estimated_state
        while(np.linalg.norm(pid.getError(current_state, wp)) > 0.05): # check the error between current state and current way point
            # calculate the current twist
            update_value = pid.update(current_state)
            # publish the twist
            pub_twist.publish(genTwistMsg(coord(update_value, current_state)))
            #print(coord(update_value, current_state))
            time.sleep(0.05)
            # update the current state
            current_state += update_value
            found_state, estimated_state = getCurrentPos(listener)
            print(found_state, estimated_state)
            if found_state:
                current_state = estimated_state
    # stop the car and exit
    pub_twist.publish(genTwistMsg(np.array([0.0,0.0,0.0])))

