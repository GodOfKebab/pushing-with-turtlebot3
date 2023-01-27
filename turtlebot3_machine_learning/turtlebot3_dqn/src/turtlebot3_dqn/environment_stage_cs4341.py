#!/usr/bin/env python
#################################################################################
# Copyright 2018 ROBOTIS CO., LTD.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#################################################################################

# Authors: Gilbert #

import rospy
import numpy as np
import math
from math import pi
from geometry_msgs.msg import Twist, Point, Pose
from sensor_msgs.msg import LaserScan
from gazebo_msgs.msg import ModelStates
from nav_msgs.msg import Odometry
from std_srvs.srv import Empty
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from .respawnGoal import Respawn
import matplotlib.pyplot as plt


class Env():
    def __init__(self, action_size):
        self.goal_x = 0
        self.goal_y = 0
        self.action_size = action_size
        self.initGoal = True
        self.position = Point()
        self.yaw = 0.
        self.cylinder_position = Point()
        self.robot_cylinder_init_dist = 9999.
        self.cylinder_goal_init_dist = 9999.
        self.pub_cmd_vel = rospy.Publisher('cmd_vel', Twist, queue_size=5)
        self.sub_odom = rospy.Subscriber('odom', Odometry, self.getOdometry)
        self.sub_cylinder = rospy.Subscriber('gazebo/model_states', ModelStates, self.getCylinderPose, queue_size=5)
        self.reset_proxy = rospy.ServiceProxy('gazebo/reset_simulation', Empty)
        self.unpause_proxy = rospy.ServiceProxy('gazebo/unpause_physics', Empty)
        self.pause_proxy = rospy.ServiceProxy('gazebo/pause_physics', Empty)
        self.respawn_goal = Respawn()

    def getOdometry(self, odom):
        self.position = odom.pose.pose.position
        orientation = odom.pose.pose.orientation
        orientation_list = [orientation.x, orientation.y, orientation.z, orientation.w]
        _, _, self.yaw = euler_from_quaternion(orientation_list)

    def getCylinderPose(self, modelStates):
        for name, pose in zip(modelStates.name, modelStates.pose):
            if name == "obstacle":
                self.cylinder_position = pose.position

    def getState(self, scan):
        scan_range = []
        min_range = 0.2
        done = False

        for i in range(len(scan.ranges)):
            if scan.ranges[i] == float('Inf'):
                scan_range.append(3.5)
            elif np.isnan(scan.ranges[i]):
                scan_range.append(0)
            else:
                scan_range.append(scan.ranges[i])

        if min_range > min(scan_range) > 0:
            done = True

        robotPos = (self.position.x, self.position.y)
        cylinderPos = (self.cylinder_position.x, self.cylinder_position.y)
        goalPos = (self.goal_x, self.goal_y)
        headingV = self.calc_heading_vector(robotPos, cylinderPos, goalPos)
        heading = math.atan2(headingV[1], headingV[0]) - self.yaw
        if heading > pi:
            heading -= 2 * pi

        elif heading < -pi:
            heading += 2 * pi

        # Uncomment below to test the heading reward vector field
        # Env.plot_heading(robotPos, cylinderPos, goalPos)

        robot_cylinder_curr_dist = math.hypot(cylinderPos[0] - robotPos[0], cylinderPos[1] - robotPos[1])
        cylinder_goal_curr_dist = math.hypot(goalPos[0] - cylinderPos[0], goalPos[1] - cylinderPos[1])

        return scan_range \
               + [heading] \
               + [robot_cylinder_curr_dist] \
               + [cylinder_goal_curr_dist], done

    def setReward(self, state, done, action):
        heading = state[-3]
        robot_cylinder_curr_dist = state[-2]
        cylinder_goal_curr_dist = state[-1]

        yaw_reward = []
        for i in range(5):
            angle = -pi / 4 + heading + (pi / 8 * i) + pi / 2
            tr = 1 - 4 * math.fabs(0.5 - math.modf(0.25 + 0.5 * angle % (2 * math.pi) / math.pi)[0])
            yaw_reward.append(tr)

        ACTION_DICT = {
            0: "Hard Left",
            1: "Soft Left",
            2: "Straight",
            3: "Soft Right",
            4: "Hard Right",
        }
        # rospy.loginfo("Best heading error is: {}".format(heading*180/math.pi))
        rospy.loginfo("Best heading action is: {}".format(ACTION_DICT[yaw_reward.index(max(yaw_reward))]))
        # print yaw_reward

        distance_rate_robot_cylinder = 1 + (self.robot_cylinder_init_dist - robot_cylinder_curr_dist) ** 2

        # dist_diff = (self.cylinder_goal_init_dist - cylinder_goal_curr_dist) / self.cylinder_goal_init_dist
        # distance_rate_cylinder_goal = 2 ** (dist_diff * 8) - 2 ** (-dist_diff * 8)

        reward = ((round(yaw_reward[action] * 5, 2)) * distance_rate_robot_cylinder)
        # rospy.loginfo("Best heading action reward is: {}".format(((round(yaw_reward[yaw_reward.index(max(yaw_reward))] * 5, 2)) * distance_rate_robot_cylinder)))
        # reward += distance_rate_cylinder_goal

        if done:
            rospy.loginfo("Collision!!")
            reward = -200
            self.pub_cmd_vel.publish(Twist())

        if cylinder_goal_curr_dist < 0.3:
            rospy.loginfo("Goal!!")
            reward = 200
            self.pub_cmd_vel.publish(Twist())
            self.goal_x, self.goal_y = self.respawn_goal.getPosition(True, delete=True)
            self.robot_cylinder_init_dist = math.hypot(self.cylinder_position.x - self.position.x,
                                                       self.cylinder_position.y - self.position.y)
            self.cylinder_goal_init_dist = math.hypot(self.goal_x - self.cylinder_position.x,
                                                      self.goal_y - self.cylinder_position.y)

        return reward

    @staticmethod
    def calc_heading_vector(robotPos, cylinderPos, goalPos):
        # STEP 1: Calculate heading between obstacle cylinder position and goal position
        goal_heading = math.atan2(goalPos[1] - cylinderPos[1], goalPos[0] - cylinderPos[0])

        # STEP 2: Apply inverse rotation -goal_heading
        x, y = Env.rotateV(robotPos[0] - cylinderPos[0], robotPos[1] - cylinderPos[1], -goal_heading)
        x = x if x != 0. else 0.001
        y = y if y != 0. else 0.001

        # STEP 3: Calculate the reward heading as if pushing from origin to the right
        if x > 0.001:
            if abs(y) < 0.20:
                y_sign = 1. if y > 0 else -1.  # preserve sign
                headingV = [-1, 1. / (10 * x * (y_sign * (abs(y) ** 0.25)))]
            else:
                headingV = [-(x ** 0.125), -y]
        else:
            headingV = [-x - 2 * (1 + x) * abs(y), 2 * -y]

        # STEP 4: Apply rotation again to get back to the initial coordinate system
        headingV[0], headingV[1] = Env.rotateV(headingV[0], headingV[1], goal_heading)

        # STEP 5: Normalize the heading vector
        headingV[0], headingV[1] = Env.normalizeV(headingV[0], headingV[1])
        return headingV[0], headingV[1]

    @staticmethod
    def normalizeV(u, v):
        return u / ((u ** 2 + v ** 2) ** 0.5), v / ((u ** 2 + v ** 2) ** 0.5)

    @staticmethod
    def rotateV(u, v, rot):
        return u * math.cos(rot) - v * math.sin(rot), u * math.sin(rot) + v * math.cos(rot)

    @staticmethod
    def plot_heading(robotPos, cylinderPos, goalPos):
        X = np.linspace(-1.1, 1.1, 30)
        Y = np.linspace(-1.1, 1.1, 30)
        # X = np.linspace(-0.3, 0.3, 30)
        # Y = np.linspace(-0.3, 0.3, 30)
        meshX, meshY = np.meshgrid(X, Y)

        U = []
        V = []
        for posY in Y:
            U.append([])
            V.append([])
            for posX in X:
                u, v = Env.calc_heading_vector((posX, posY), cylinderPos, goalPos)
                U[-1].append(u)
                V[-1].append(v)

        labels = np.around(np.arange(-1.1, 1.2, 0.1), decimals=1)
        # labels = np.around(np.arange(-0.3, 0.33, 0.03), decimals=1)
        fig = plt.figure(1)
        ax = fig.add_subplot(111)
        plt.grid()
        plt.quiver(meshX, meshY, U, V)
        plt.xticks(labels)
        ax.set_xticklabels(labels, rotation=45)
        ax.set_aspect('equal', adjustable='box')
        plt.yticks(labels)
        plt.plot([robotPos[0]], [robotPos[1]], 'mo', markersize=15)
        plt.plot([cylinderPos[0]], [cylinderPos[1]], 'bo', markersize=15)
        plt.plot([goalPos[0]], [goalPos[1]], 'go', markersize=15)
        # plt.plot([0], [0], 'bo', markersize=60)
        plt.show()

    def step(self, action):
        max_angular_vel = 1.5
        ang_vel = ((self.action_size - 1) / 2 - action) * max_angular_vel * 0.5

        vel_cmd = Twist()
        vel_cmd.linear.x = 0.15
        vel_cmd.angular.z = ang_vel
        # vel_cmd.linear.x = 0
        # vel_cmd.angular.z = 0
        self.pub_cmd_vel.publish(vel_cmd)

        data = None
        while data is None:
            try:
                data = rospy.wait_for_message('scan', LaserScan, timeout=5)
            except:
                pass

        state, done = self.getState(data)
        reward = self.setReward(state, done, action)

        return np.asarray(state), reward, done

    def reset(self):
        rospy.wait_for_service('gazebo/reset_simulation')
        try:
            self.reset_proxy()
        except (rospy.ServiceException) as e:
            print("gazebo/reset_simulation service call failed")

        data = None
        while data is None:
            try:
                data = rospy.wait_for_message('scan', LaserScan, timeout=5)
            except:
                pass

        if self.initGoal:
            self.goal_x, self.goal_y = self.respawn_goal.getPosition()
            self.initGoal = False

        self.robot_cylinder_init_dist = math.hypot(self.cylinder_position.x - self.position.x,
                                                   self.cylinder_position.y - self.position.y)
        self.cylinder_goal_init_dist = math.hypot(self.goal_x - self.cylinder_position.x,
                                                  self.goal_y - self.cylinder_position.y)
        # rospy.loginfo("robot_x_init={}, robot_y_init={}, cylinder_x_init={}, cylinder_y_init={}".format(self.robot_x_init, self.robot_y_init, self.cylinder_x_init, self.cylinder_y_init))
        state, done = self.getState(data)

        return np.asarray(state)
