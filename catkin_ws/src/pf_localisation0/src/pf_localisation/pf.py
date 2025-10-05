#!/usr/bin/env python3
import random
import math
import numpy as np
from geometry_msgs.msg import PoseArray, Pose
from pf_localisation.pf_base import PFLocaliserBase
from pf_localisation.util import rotateQuaternion, getHeading


class PFLocaliser(PFLocaliserBase):
       
    def __init__(self):
        # Call the superclass constructor
        super(PFLocaliser, self).__init__()
        
        # Set motion model parameters
        self.ODOM_ROTATION_NOISE = 0.1
        self.ODOM_TRANSLATION_NOISE = 0.1
        self.ODOM_DRIFT_NOISE = 0.1
 
        # Sensor model parameters
        self.NUMBER_PREDICTED_READINGS = 20 	# Number of readings to predict
        
       
    def initialise_particle_cloud(self, initialpose):

        '''
        This should instantiate and return a PoseArray [3] object, which contains 
        a list of Pose objects. Each of these Poses should be set to a random position and 
        orientation around the initial pose, e.g. by adding a Gaussian random number multiplied by a noise 
        parameter to the initial pose.
        '''
        particle_count = 100  # Number of particles (can be adjusted)
        noise = 0.2  # Noise level for initialization

        particle_cloud = PoseArray()
        particle_cloud.header = initialpose.header

        for _ in range(particle_count):
            # Generate random x, y, and yaw values around the initial pose
            x = initialpose.pose.pose.position.x + random.gauss(0, noise)
            y = initialpose.pose.pose.position.y + random.gauss(0, noise)
            yaw = getHeading(initialpose.pose.pose.orientation) + random.gauss(0, noise)

            # Create a new pose
            pose = Pose()
            pose.position.x = x
            pose.position.y = y
            pose.orientation = rotateQuaternion(initialpose.pose.pose.orientation, yaw)

            particle_cloud.poses.append(pose)

        self.particlecloud = particle_cloud
        return particle_cloud
 
    
    def update_particle_cloud(self, scan):
        weights = []
        for pose in self.particlecloud.poses:
            weight = self.sensor_model.get_weight(scan, pose)
            weights.append(weight)

        # Normalize weights
        total_weight = sum(weights)
        if total_weight > 0:
            weights = [w / total_weight for w in weights]
        else:
            weights = [1.0 / len(weights)] * len(weights)

        # Resample particles using the normalized weights
        new_particles = []
        for _ in range(len(self.particlecloud.poses)):
            index = np.random.choice(range(len(self.particlecloud.poses)), p=weights)
            selected_particle = self.particlecloud.poses[index]

            # Add Gaussian noise to the selected particle
            new_pose = Pose()
            new_pose.position.x = selected_particle.position.x + random.gauss(0, 0.05)
            new_pose.position.y = selected_particle.position.y + random.gauss(0, 0.05)
            new_pose.orientation = rotateQuaternion(
                selected_particle.orientation, random.gauss(0, 0.05)
            )

            new_particles.append(new_pose)

        self.particlecloud.poses = new_particles


    def estimate_pose(self):
        if not self.particlecloud.poses:
            return None

        avg_x = sum(pose.position.x for pose in self.particlecloud.poses) / len(self.particlecloud.poses)
        avg_y = sum(pose.position.y for pose in self.particlecloud.poses) / len(self.particlecloud.poses)

        # Average quaternion
        avg_quaternion = Pose().orientation
        avg_quaternion.x = sum(pose.orientation.x for pose in self.particlecloud.poses) / len(self.particlecloud.poses)
        avg_quaternion.y = sum(pose.orientation.y for pose in self.particlecloud.poses) / len(self.particlecloud.poses)
        avg_quaternion.z = sum(pose.orientation.z for pose in self.particlecloud.poses) / len(self.particlecloud.poses)
        avg_quaternion.w = sum(pose.orientation.w for pose in self.particlecloud.poses) / len(self.particlecloud.poses)

        estimated_pose = Pose()
        estimated_pose.position.x = avg_x
        estimated_pose.position.y = avg_y
        estimated_pose.orientation = avg_quaternion

        return estimated_pose

