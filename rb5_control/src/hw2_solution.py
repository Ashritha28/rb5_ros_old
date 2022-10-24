#!/usr/bin/env python
import sys
import rospy
from geometry_msgs.msg import Twist
from april_detection.msg import Pose
import numpy as np

"""
The class of the pid controller.
"""

def isRotationMatrix(R) :
    Rt = np.transpose(R)
    shouldBeIdentity = np.dot(Rt, R)
    I = np.identity(3, dtype = R.dtype)
    n = np.linalg.norm(I - shouldBeIdentity)
    return n < 1e-6
 
# Calculates rotation matrix to euler angles
# The result is the same as MATLAB except the order
# of the euler angles ( x and z are swapped ).
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
    print(np.array([x, y, z]))
    return np.array([x, y, z])

class PIDcontroller:
    def __init__(self, Kp, Ki, Kd):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.target = None
        self.I = np.array([0.0,0.0,0.0])
        self.lastError = np.array([0.0,0.0,0.0])
        self.timestep = 0.1
        self.maximumValue = 0.1

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
    

    def planner(msg):

        waypoint = np.array([[0.0,0.0,0.0], 
                    [1.0,0.0,0.0],
                    [1.0,1.0,np.pi],
                    [0.0,0.0,0.0]]) 

        # init pid controller
        pid = PIDcontroller(0.02,0.005,0.005)

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
            while(np.linalg.norm(pid.getError(current_state, wp)) > 0.05): # check the error between current state and current way point
                # calculate the current twist
                update_value = pid.update(current_state)
                # publish the twist
                pub_twist.publish(genTwistMsg(coord(update_value, current_state)))
                #print(coord(update_value, current_state))
                time.sleep(0.05)

                if msg.pose:
                    cur_pose_matrix = np.asarray(msg.pose.matrix)
                    trans = cur_pose_matrix[:3, 3]
                    print("Translation:", trans)
                    rot = cur_pose_matrix[:3, :3]
                    print("Rotation part of pose:", rot)
                    rot_y = rotationMatrixToEulerAngles(rot)[1]

                    cur_pose = [trans[0], trans[1], rot_y]

                    update_value = pid.update(cur_pose)

                # update the current state
                current_state += update_value
        # stop the car and exit
        pub_twist.publish(genTwistMsg(np.array([0.0,0.0,0.0])))



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
    J = np.array([[np.cos(current_state[2]), np.sin(current_state[2]), 0.0],
                  [-np.sin(current_state[2]), np.cos(current_state[2]), 0.0],
                  [0.0,0.0,1.0]])
    return np.dot(J, twist)
    


if __name__ == "__main__":
    import time
    rospy.init_node("hw2")
    pub_twist = rospy.Publisher("/twist", Twist, queue_size=1)
    rospy.Subscriber('/current_pose', Pose, planner) 
    

