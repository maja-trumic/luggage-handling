#This code is similar to OpenAI gym etedal.net/2020/04/pybullet-panda.html

import pybullet as p
import time
import math
import matplotlib.pyplot as plt
import numpy as np

# Connect to the PyBullet physics server
physicsClient = p.connect(p.GUI)

# Set gravity
p.setGravity(0, 0, -9.81)

# Create a ground plane
planeId = p.createCollisionShape(p.GEOM_PLANE)
p.createMultiBody(0, planeId)

# Set the desired time step for the simulation
desired_time_step = 1.0 / 240.0
p.setPhysicsEngineParameter(fixedTimeStep=desired_time_step)

# Create a cylinder (cylinder represents the base)
cylinderRadius = 1.4224
cylinderHeight = 0.3048
cylinderPosition = [0, 0, cylinderHeight / 2]  # Place the cylinder slightly above the ground
cylinderId = p.createCollisionShape(p.GEOM_CYLINDER, radius=cylinderRadius, height=cylinderHeight)
cylinderVisualId = p.createVisualShape(p.GEOM_CYLINDER, radius=cylinderRadius, length=cylinderHeight, rgbaColor=[0.8, 0.8, 0.8, 1])
cylinderBodyId = p.createMultiBody(baseMass=0, baseCollisionShapeIndex=cylinderId, baseVisualShapeIndex=cylinderVisualId, basePosition=cylinderPosition)

# Set the initial angular velocity to make it rotate
p.resetBaseVelocity(cylinderBodyId, linearVelocity=[0, 0, 0], angularVelocity=[0, 0, 0.35])
#p.resetBaseVelocity(inner_body, linearVelocity=[0, 0, 0], angularVelocity=[0, 0, 0.35])


boxHalfExtents = [0.25, 0.35, 0.1]
boxPosition = [0, -0.8, 0.01+cylinderHeight + boxHalfExtents[2]]  # Place the box on top of the cylinder
boxId = p.createCollisionShape(p.GEOM_BOX, halfExtents=boxHalfExtents)
boxVisualId = p.createVisualShape(p.GEOM_BOX, halfExtents=boxHalfExtents, rgbaColor=[0.2, 0.2, 0.8, 1])
boxBodyId = p.createMultiBody(baseMass=15, baseCollisionShapeIndex=boxId, baseVisualShapeIndex=boxVisualId, basePosition=boxPosition)

# Attach four wheels to the luggage
wheel_ids = []
wheel_radius = 0.05
wheel_height = 0.05
wheel_positions = [(-0.25, -0.35, -0.1), (0.25, -0.35, -0.1), (-0.25, -0.35, 0.1), (0.25, -0.35, 0.1)]
higher_friction_value = 5.0 # for the higher value, the robot does not kick so much the luggage

for position in wheel_positions:
    wheel_cylinder = p.createCollisionShape(p.GEOM_CYLINDER, radius=wheel_radius, height=wheel_height)
    wheel_visual = p.createVisualShape(p.GEOM_CYLINDER, radius=wheel_radius, length=wheel_height, rgbaColor=[0.8, 0.1, 0.1, 1])
    wheel_body = p.createMultiBody(baseMass=0.1, baseCollisionShapeIndex=wheel_cylinder, baseVisualShapeIndex=wheel_visual, basePosition=position)
    wheel_ids.append(wheel_body)
    # Attach wheel to the luggage using a fixed joint
    p.createConstraint(parentBodyUniqueId=boxBodyId, parentLinkIndex=-1, childBodyUniqueId=wheel_body, childLinkIndex=-1,
                       jointType=p.JOINT_FIXED, jointAxis=[0, 0, 0], parentFramePosition=position, # here should come the rotational joint
                       childFramePosition=[0, 0, 0], childFrameOrientation=[0, 0, 0, 1])
    #p.changeDynamics(wheel_body, -1, lateralFriction=higher_friction_value)


p.resetDebugVisualizerCamera(cameraDistance=2, cameraYaw=-130, cameraPitch=-40, cameraTargetPosition=[0.7, 0, 0])

# Set up camera parameters
width = 640
height = 480
fov = 60
aspect_ratio = width / height
near_plane = 0.01
far_plane = 100

# Set up camera position and orientation
camera_target_position = [0.7, 0, 0]
camera_distance = 2
camera_yaw = -130
camera_pitch = -40

num_frames = 1000
for i in range(num_frames):
    p.configureDebugVisualizer(p.COV_ENABLE_SINGLE_STEP_RENDERING) 

    # Update camera position and orientation
    view_matrix = p.computeViewMatrixFromYawPitchRoll(
        camera_target_position, camera_distance, camera_yaw, camera_pitch, 0, upAxisIndex=2)
    projection_matrix = p.computeProjectionMatrixFOV(
        fov, aspect_ratio, near_plane, far_plane)
    
    # Get camera image
    img = p.getCameraImage(width, height, view_matrix, projection_matrix)
    
    # Reshape the image data
    img_data = np.reshape(img[2], (height, width, 4))  # RGBA format
    
    # Display the image using Matplotlib
    if i==100 or i==200 or i==300 or i==400 or i==500 or i==600 or i==700 or i==800:
        plt.imshow(img_data)
        plt.axis('off')  # Hide axis
        plt.savefig(f'image_{i}.png', bbox_inches='tight', pad_inches=0)  # Save the image as PNG
        plt.close()  # Close the plot to avoid memory leak

    
    p.stepSimulation()
    #time.sleep(1.0 / 240.0)  # Control simulation speed

sys.exit()




