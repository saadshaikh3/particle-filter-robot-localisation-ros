"""
@author rowanms
An abstract Localiser which needs to be extended as PFLocaliser
before PFLocalisationNode will work.
@author burbrcjc
Converted to Python 3.8 for ROS Noetic
"""

import rospy
from geometry_msgs.msg import (PoseWithCovarianceStamped, PoseArray,
                               Quaternion, Transform, TransformStamped)
from tf.msg import tfMessage
from tf import transformations
from nav_msgs.msg import OccupancyGrid

import math
import random
import numpy as np
from pf_localisation.util import rotateQuaternion, getHeading
from threading import Lock
import time
import pf_localisation.sensor_model as sensor_model

PI_OVER_TWO = math.pi / 2  # For faster calculations


class PFLocaliserBase:
    INIT_X = 10  # Initial x location of robot (metres)
    INIT_Y = 5  # Initial y location of robot (metres)
    INIT_Z = 0  # Initial z location of robot (metres)
    INIT_HEADING = 0  # Initial orientation of robot (radians)

    def __init__(self):
        self.estimatedpose = PoseWithCovarianceStamped()
        self.occupancy_map = OccupancyGrid()
        self.particlecloud = PoseArray()
        self.tf_message = tfMessage()

        self._update_lock = Lock()

        # Parameters
        self.ODOM_ROTATION_NOISE = 0  # Odometry model rotation noise
        self.ODOM_TRANSLATION_NOISE = 0  # Odometry x-axis (forward) noise
        self.ODOM_DRIFT_NOISE = 0  # Odometry y-axis (side-side) noise
        self.NUMBER_PREDICTED_READINGS = 20

        self.prev_odom_x = 0.0
        self.prev_odom_y = 0.0
        self.prev_odom_heading = 0.0
        self.last_odom_pose = None

        self.odom_initialised = False
        self.sensor_model_initialised = False

        self.estimatedpose.pose.pose.position.x = self.INIT_X
        self.estimatedpose.pose.pose.position.y = self.INIT_Y
        self.estimatedpose.pose.pose.position.z = self.INIT_Z
        self.estimatedpose.pose.pose.orientation = rotateQuaternion(Quaternion(w=1.0), self.INIT_HEADING)

        self.estimatedpose.header.frame_id = "map"
        self.particlecloud.header.frame_id = "map"

        self.sensor_model = sensor_model.SensorModel()

    def initialise_particle_cloud(self, initialpose):
        raise NotImplementedError()

    def update_filter(self, scan):
        if not self.sensor_model_initialised:
            self.sensor_model.set_laser_scan_parameters(
                self.NUMBER_PREDICTED_READINGS,
                scan.range_max,
                len(scan.ranges),
                scan.angle_min,
                scan.angle_max,
            )
            self.sensor_model_initialised = True

        with self._update_lock:
            t = time.time()
            self.update_particle_cloud(scan)
            self.particlecloud.header.frame_id = "map"
            self.estimatedpose.pose.pose = self.estimate_pose()
            current_time = rospy.Time.now()
            self.recalculate_transform(current_time)
            self.particlecloud.header.stamp = current_time
            self.estimatedpose.header.stamp = current_time

        return time.time() - t

    def update_particle_cloud(self, scan):
        raise NotImplementedError()

    def estimate_pose(self):
        raise NotImplementedError()

    def recalculate_transform(self, current_time):
        transform = Transform()

        T_est = transformations.quaternion_matrix([
            self.estimatedpose.pose.pose.orientation.x,
            self.estimatedpose.pose.pose.orientation.y,
            self.estimatedpose.pose.pose.orientation.z,
            self.estimatedpose.pose.pose.orientation.w,
        ])
        T_est[0, 3] = self.estimatedpose.pose.pose.position.x
        T_est[1, 3] = self.estimatedpose.pose.pose.position.y
        T_est[2, 3] = self.estimatedpose.pose.pose.position.z

        T_odom = transformations.quaternion_matrix([
            self.last_odom_pose.pose.pose.orientation.x,
            self.last_odom_pose.pose.pose.orientation.y,
            self.last_odom_pose.pose.pose.orientation.z,
            self.last_odom_pose.pose.pose.orientation.w,
        ])
        T_odom[0, 3] = self.last_odom_pose.pose.pose.position.x
        T_odom[1, 3] = self.last_odom_pose.pose.pose.position.y
        T_odom[2, 3] = self.last_odom_pose.pose.pose.position.z

        T = np.dot(T_est, np.linalg.inv(T_odom))
        q = transformations.quaternion_from_matrix(T)

        transform.translation.x = T[0, 3]
        transform.translation.y = T[1, 3]
        transform.translation.z = T[2, 3]
        transform.rotation.x = q[0]
        transform.rotation.y = q[1]
        transform.rotation.z = q[2]
        transform.rotation.w = q[3]

        new_tfstamped = TransformStamped()
        new_tfstamped.child_frame_id = "odom"
        new_tfstamped.header.frame_id = "map"
        new_tfstamped.header.stamp = current_time
        new_tfstamped.transform = transform

        self.tf_message = tfMessage(transforms=[new_tfstamped])

    def predict_from_odometry(self, odom):
        with self._update_lock:
            t = time.time()
            x = odom.pose.pose.position.x
            y = odom.pose.pose.position.y
            new_heading = getHeading(odom.pose.pose.orientation)

            if not self.odom_initialised:
                self.prev_odom_x = x
                self.prev_odom_y = y
                self.prev_odom_heading = new_heading
                self.odom_initialised = True

            dif_x = x - self.prev_odom_x
            dif_y = y - self.prev_odom_y
            dif_heading = new_heading - self.prev_odom_heading
            if dif_heading > math.pi:
                dif_heading -= 2 * math.pi
            elif dif_heading < -math.pi:
                dif_heading += 2 * math.pi

            self.prev_odom_x = x
            self.prev_odom_y = y
            self.prev_odom_heading = new_heading
            self.last_odom_pose = odom

            distance_travelled = math.sqrt(dif_x**2 + dif_y**2)
            direction_travelled = math.atan2(dif_y, dif_x)
            if abs(new_heading - direction_travelled) > PI_OVER_TWO:
                distance_travelled *= -1

            for p in self.particlecloud.poses:
                rnd = random.normalvariate(0, 1)
                p.orientation = rotateQuaternion(
                    p.orientation,
                    dif_heading + rnd * dif_heading * self.ODOM_ROTATION_NOISE,
                )
                theta = getHeading(p.orientation)
                travel_x = distance_travelled * math.cos(theta)
                travel_y = distance_travelled * math.sin(theta)
                p.position.x += travel_x + rnd * travel_x * self.ODOM_TRANSLATION_NOISE
                p.position.y += travel_y + rnd * travel_y * self.ODOM_DRIFT_NOISE

        return time.time() - t

    def set_initial_pose(self, pose):
        self.estimatedpose.pose = pose.pose
        rospy.loginfo("Got pose. Calling initialise_particle_cloud().")
        self.particlecloud = self.initialise_particle_cloud(self.estimatedpose)
        self.particlecloud.header.frame_id = "map"

    def set_map(self, occupancy_map):
        self.occupancy_map = occupancy_map
        self.sensor_model.set_map(occupancy_map)
        rospy.loginfo("Particle filter got map. (Re)initialising.")
        self.particlecloud = self.initialise_particle_cloud(self.estimatedpose)
        self.particlecloud.header.frame_id = "map"
