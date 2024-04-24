import pybullet as p
import time
import matplotlib.pyplot as plt
import numpy as np

# Connect to the PyBullet Physics Server
physicsClient = p.connect(p.GUI)
p.setGravity(0, 0, -9.81)  # Set gravity

# Create a ground plane
planeId = p.createCollisionShape(p.GEOM_PLANE)
p.createMultiBody(0, planeId)

# Create the conveyor belt segments
num_segments = 20
segment_length = 1.0
segment_width = 1.016 #33 inches is the width of the conveyor part, 40 inches is the total width
segment_height = 0.3048 # 12 inches
spacing = 0. # Spacing between segments

segments = []

for i in range(num_segments):
    # Create the segment
    segment_id = p.createCollisionShape(p.GEOM_BOX, halfExtents=[segment_length / 2, segment_width / 2, segment_height / 2])
    segment_position = [(i-2) * (segment_length + spacing), 0, segment_height / 2 + 0.]
    segment_orientation = p.getQuaternionFromEuler([0, 0, 0])  # No rotation
    segment_body_id = p.createMultiBody(baseMass=0, baseCollisionShapeIndex=segment_id,
                                        basePosition=segment_position, baseOrientation=segment_orientation)
    segments.append(segment_body_id)
    # Change the color of the segment
    p.changeVisualShape(segment_body_id, -1, rgbaColor=[0.8, 0.8, 0.8, 1])  # Red color

    # Change the friction of the segment
    p.changeDynamics(segment_body_id, -1, lateralFriction=1.0)  # Adjust friction value, default is 1.0

    # Set up the conveyor belt speed
    p.resetBaseVelocity(segment_id, linearVelocity=[0.5, 0, 0], angularVelocity=[0, 0, 0])
    #p.setJointMotorControl2(bodyIndex=segment_body_id, jointIndex=0, controlMode=p.VELOCITY_CONTROL, targetVelocity = 0.5, force = 0)



boxHalfExtents = [0.25, 0.35, 0.1]
boxPosition = [1.5, 0, 0.0 + segment_height + boxHalfExtents[2]]  # Place the box on top of the cylinder
boxId = p.createCollisionShape(p.GEOM_BOX, halfExtents=boxHalfExtents)
boxVisualId = p.createVisualShape(p.GEOM_BOX, halfExtents=boxHalfExtents, rgbaColor=[0.2, 0.2, 0.8, 1])
boxBodyId = p.createMultiBody(baseMass=15, baseCollisionShapeIndex=boxId, baseVisualShapeIndex=boxVisualId, basePosition=boxPosition)
p.changeDynamics(boxBodyId, -1, lateralFriction=2.0)

# Attach four wheels to the luggage
wheel_ids = []
wheel_radius = 0.05
wheel_height = 0.05
wheel_positions = [(-0.25, -0.35, -0.1), (0.25, -0.35, -0.1), (-0.25, -0.35, 0.1), (0.25, -0.35, 0.1)]
higher_friction_value = 0.0 # for the higher value, the robot does not kick so much the luggage

for position in wheel_positions:
    wheel_cylinder = p.createCollisionShape(p.GEOM_CYLINDER, radius=wheel_radius, height=wheel_height)
    wheel_visual = p.createVisualShape(p.GEOM_CYLINDER, radius=wheel_radius, length=wheel_height, rgbaColor=[0.8, 0.1, 0.1, 1])
    wheel_body = p.createMultiBody(baseMass=0.01, baseCollisionShapeIndex=wheel_cylinder, baseVisualShapeIndex=wheel_visual, basePosition=position)
    wheel_ids.append(wheel_body)
    # Attach wheel to the luggage using a fixed joint
    p.createConstraint(parentBodyUniqueId=boxBodyId, parentLinkIndex=-1, childBodyUniqueId=wheel_body, childLinkIndex=-1,
                       jointType=p.JOINT_FIXED, jointAxis=[0, 0, 0], parentFramePosition=position, # here should come the rotational joint
                       childFramePosition=[0, 0, 0], childFrameOrientation=[0, 0, 0, 1])
    #p.changeDynamics(wheel_body, -1, lateralFriction=higher_friction_value)

p.resetDebugVisualizerCamera(cameraDistance=5, cameraYaw=-40, cameraPitch=-30, cameraTargetPosition=[3, 0, 0])

# Set up camera parameters
width = 640
height = 480
fov = 60
aspect_ratio = width / height
near_plane = 0.01
far_plane = 100

# Set up camera position and orientation
camera_target_position = [3, 0, 0]
camera_distance = 3
camera_yaw = -40
camera_pitch = -30

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
        plt.savefig(f'image_lin_{i}.png', bbox_inches='tight', pad_inches=0)  # Save the image as PNG
        plt.close()  # Close the plot to avoid memory leak

    
    p.stepSimulation()
    #time.sleep(1.0 / 240.0)  # Control simulation speed

sys.exit()