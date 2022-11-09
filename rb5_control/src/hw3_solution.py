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
from numpy.linalg import inv, multi_dot

"""
The class of the pid controller.
"""

##### TODO
# Figure out how to get H matrix
# When should I take measurement?
# When I get tag in robot frame, how to convert to WC?
# Write uncertainity (sigma) after every timestep into a file
# Modify PID controller

class KalmanFilter:
    def __init__(self):
        self.robot_id = 10
        self.s = {
            10: [0.0,0.0,0.0]
        }
        self.sigma = np.array([[0.1, 0.0, 0.0],
              [0.0, 0.1, 0.0],
              [0.0, 0.0, 0.01]])
        self.R = np.array([[0.01, 0.0, 0.0],
              [0.0, 0.01, 0.0],
              [0.0, 0.0, 0.001]])
        self.Q = np.array([[0.03, 0.0, 0.0],
              [0.0, 0.03, 0.0],
              [0.0, 0.0, 0.003]])
        self.H = np.array([[0.0, 0.0, 0.0],
              [0.0, 0.0, 0.0],
              [0.0, 0.0, 0.0]])
        self.zt = np.array([[0.0],
                           [0.0],
                           [0.0]])
        self.seen_ids = []


    def predict_state(self, prev_state, Gu):
        print("Previous state: ", prev_state)
        print("Control update: ", Gu)
        self.s[self.robot_id] = prev_state + Gu
        print("Prediction: ", self.s)


    def predict_sigma(self):
        self.sigma = self.sigma + self.Q

    def update_state_after_measurement(self, id, landmark_r):
        return None

    def restack_sigma(self):
        sigma_shape = np.shape(self.sigma)
        top_matrix = np.hstack((self.sigma, np.zeros((sigma_shape[0],3))))
        bottom_matrix = np.hstack((np.zeros((3, sigma_shape[1])), self.sigma[0:3, 0:3] + self.R[0:3, 0:3]))
        self.sigma = np.vstack((top_matrix, bottom_matrix))

    def restack_Q(self):
        q_shape = np.shape(self.Q)
        top_matrix = np.hstack((self.Q, np.zeros((q_shape[0], 3))))
        bottom_matrix = np.hstack((np.zeros((3, q_shape[1])), self.Q[0:3, 0:3]))
        self.Q = np.vstack((top_matrix, bottom_matrix))

    def restack_R(self):
        r_shape = np.shape(self.R)
        top_matrix = np.hstack((self.R, np.zeros((r_shape[0], 3))))
        bottom_matrix = np.hstack((np.zeros((3, r_shape[1])), self.R[0:3, 0:3]))
        self.R = np.vstack((top_matrix, bottom_matrix))

    def compute_kalman_gain(self):
        S = inv(np.matmul(np.matmul(self.H, self.sigma), np.transpose(self.H)) + self.R)
        self.K = np.matmul(np.matmul(self.sigma, np.transpose(self.H)), S)

    def compute_H(self):
        return None

    def update_state(self):
        self.s = self.s + np.matmul(self.K, self.zt - np.matmul(self.H, self.s))

    def update_sigma(self):
        KH = np.matmul(self.K, self.H)
        I = np.eye(np.shape(KH)[0])
        self.sigma = np.matmul((I - KH), self.sigma)


class PIDcontroller:
    def __init__(self, Kp, Ki, Kd):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.target = None
        self.I = np.array([0.0, 0.0, 0.0])
        self.lastError = np.array([0.0, 0.0, 0.0])
        self.timestep = 0.1
        self.maximumValue = 0.02

    def setTarget(self, targetx, targety, targetw):
        """
        set the target pose.
        """
        self.I = np.array([0.0, 0.0, 0.0])
        self.lastError = np.array([0.0, 0.0, 0.0])
        self.target = np.array([targetx, targety, targetw])

    def setTarget(self, state):
        """
        set the target pose.
        """
        self.I = np.array([0.0, 0.0, 0.0])
        self.lastError = np.array([0.0, 0.0, 0.0])
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
        if (resultNorm > self.maximumValue):
            result = (result / resultNorm) * self.maximumValue
            self.I = 0.0

        return result


def getMeasurement(l, kf):
    """
    Given the tf listener, we consider the camera's z-axis is the header of the car
    """
    br = tf.TransformBroadcaster()
    result = None
    foundSolution = False

    for i in range(0, 9):
        camera_name = "camera_" + str(i)
        if l.frameExists(camera_name):
            try:
                now = rospy.Time()
                # wait for the transform ready from the map to the camera for 1 second.
                l.waitForTransform("map", camera_name, now, rospy.Duration(1.0))
                # extract the transform camera pose in the map coordinate.
                (trans, rot) = l.lookupTransform("map", camera_name, now)
                # convert the rotate matrix to theta angle in 2d
                matrix = quaternion_matrix(rot)
                angle = math.atan2(matrix[1][2], matrix[0][2])
                # this is not required, I just used this for debug in RVIZ
                br.sendTransform((trans[0], trans[1], 0), tf.transformations.quaternion_from_euler(0, 0, angle),
                                 rospy.Time.now(), "base_link", "map")
                result = np.array([trans[0], trans[1], angle])
                kf.seen_ids.append(i)
                kf.update_state_after_measurement(i, result)
                foundSolution = True
                #break - Do not break, take as many measurements as possible
            except (
            tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException, tf2_ros.TransformException):
                print("meet error")
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
                  [0.0, 0.0, 1.0]])
    return np.dot(J, twist)


if __name__ == "__main__":
    import time

    rospy.init_node("hw2")
    pub_twist = rospy.Publisher("/twist", Twist, queue_size=1)

    listener = tf.TransformListener()

    waypoint = np.array([[0.0, 0.0, 0.0],
                         [1.0, 0.0, 0.0],
                         [1.0, 2.0, np.pi],
                         [0.0, 0.0, 0.0]])

    # init pid controller
    pid = PIDcontroller(0.0185, 0.0015, 0.09)

    # init current state
    current_state = np.array([0.0, 0.0, 0.0])

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
        # print(coord(update_value, current_state))
        time.sleep(0.05)
        # update the current state
        current_state += update_value
        found_state, estimated_state = getCurrentPos(listener)
        if found_state:  # if the tag is detected, we can use it to update current state.
            current_state = estimated_state
        while (np.linalg.norm(
                pid.getError(current_state, wp)) > 0.05):  # check the error between current state and current way point
            # calculate the current twist
            update_value = pid.update(current_state)
            # publish the twist
            pub_twist.publish(genTwistMsg(coord(update_value, current_state)))
            # print(coord(update_value, current_state))
            time.sleep(0.05)
            # update the current state
            current_state += update_value
            found_state, estimated_state = getCurrentPos(listener)
            if found_state:
                current_state = estimated_state
    # stop the car and exit
    pub_twist.publish(genTwistMsg(np.array([0.0, 0.0, 0.0])))

