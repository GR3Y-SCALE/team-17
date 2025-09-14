#!/usr/bin/python

# Import the warehouse bot library
from warehousebot_lib import *

# Import additional modules
import os,sys,time
from collections import namedtuple
# my stuff
sys.path.append(os.path.join(os.path.dirname(__file__), '..'))
from navigation.NavClass import NavClass

Obstacle = namedtuple('Obstacle', ['distance_to_robot', 'degree'])

def clear_screen():
	"""Clear the terminal screen for better output readability"""
	os.system('cls' if os.name == 'nt' else 'clear')

# def driveDistance(x, rot):
# 	robotHandle = warehouseBotSim.robotHandle
# 	initialPos = warehouseBotSim.getRobotPosition()
# 	initialOrientation = warehouseBotSim.getRobotOrientation()
# 	distanceTravelled = 0
# 	angleTurned = 0
# 	warehouseBotSim.SetTargetVelocities(x, rot)
# 	while distanceTravelled < abs(x):

# CONFIGURE SCENE PARAMETERS
sceneParameters = SceneParameters()

# Set which items appear at which picking stations (0=station 1, 1=station 2, 2=station 3)
# Set to -1 to leave a station empty
sceneParameters.pickingStationContents[0] = warehouseObjects.bowl    # Bowl at picking station 1
sceneParameters.pickingStationContents[1] = warehouseObjects.mug     # Mug at picking station 2  
sceneParameters.pickingStationContents[2] = warehouseObjects.bottle  # Bottle at picking station 3

# Set obstacle starting positions [x, y] in meters
# Use -1 to keep current CoppeliaSim position, None to disable obstacle
sceneParameters.obstacle0_StartingPosition = [-0.2, -0.25]
sceneParameters.obstacle1_StartingPosition = -1  # Use current position
sceneParameters.obstacle2_StartingPosition = -1  # Use current position

# CONFIGURE ROBOT PARAMETERS
robotParameters = RobotParameters()

# Drive system settings
robotParameters.driveType = 'differential'        # Type of drive system
robotParameters.minimumLinearSpeed = 0.0          # Minimum forward speed (m/s)
robotParameters.maximumLinearSpeed = 0.25         # Maximum forward speed (m/s)
robotParameters.driveSystemQuality = 1            # Drive quality (0-1, 1=perfect)

# Camera settings
robotParameters.cameraOrientation = 'landscape'   # Camera orientation
robotParameters.cameraDistanceFromRobotCenter = 0.1  # Distance from robot center (m)
robotParameters.cameraHeightFromFloor = 0.15      # Height above floor (m)
robotParameters.cameraTilt = 0.0                  # Camera tilt angle (radians)

# Object detection ranges (in meters)
robotParameters.maxItemDetectionDistance = 1.0         # Items
robotParameters.maxPackingBayDetectionDistance = 2.5   # Picking stations
robotParameters.maxObstacleDetectionDistance = 1.5     # Obstacles
robotParameters.maxRowMarkerDetectionDistance = 2.5    # Row markers

# Item collection settings
robotParameters.collectorQuality = 1              # Collector reliability (0-1)
robotParameters.maxCollectDistance = 0.15         # Maximum collection distance (m)

# Simulation settings
robotParameters.sync = False  # Use asynchronous mode (recommended)

# MAIN PROGRAM
if __name__ == '__main__':
	# Use try-except to handle Ctrl+C gracefully
	try:
		print("EGB320 CoppeliaSim Warehouse Robot")
		print("Press Ctrl+C to stop the simulation\n")
		
		# Enable/disable debug output
		show_debug_info = True

		# Create and initialize the warehouse robot
		print("Connecting to CoppeliaSim...")
		warehouseBotSim = COPPELIA_WarehouseRobot(robotParameters, sceneParameters, 
													coppelia_server_ip='127.0.0.1', port=23000)
		
		# Start the simulation
		warehouseBotSim.StartSimulator()
		navigation = NavClass(60)

		# Main control loop
		print("Starting main control loop...")
		print("Robot is now ready for navigation commands.")
		
		goal_deg = None
		warehouseBotSim.SetTargetVelocities(0.0, 0.0)  # Stop the robot
		dt = 0.1  # desired step size (50 ms)
		last_time = time.time()

		forward_vel = 0.0
		rot_vel = 0.0
		while True:
			now = time.time()
			elapsed = now - last_time
			if elapsed < dt:
				time.sleep(dt - elapsed)
				continue
			last_time = now
			

			# Get all detected objects in camera field of view
			objectsRB = warehouseBotSim.GetDetectedObjects([
				warehouseObjects.items,                # Items (bowls, mugs, etc.)
				warehouseObjects.shelves,              # Storage shelves
				warehouseObjects.row_markers,          # Navigation markers
				warehouseObjects.obstacles,            # Obstacles to avoid
				warehouseObjects.pickingStation,       # Main picking station
				warehouseObjects.PickingStationMarkers # Individual picking stations
			])

			# Unpack the detection results
			itemsRB, packingStationRB, obstaclesRB, rowMarkerRB, shelfRB, pickingStationRB = objectsRB

			# Optional: Get camera image for computer vision processing
			# resolution, image_data = warehouseBotSim.GetCameraImage()

			# Clear screen and show current status
			if show_debug_info:
				clear_screen()
				print("EGB320 Warehouse Robot - Object Detection Status")
				print("=" * 50)
				
				# Display detected objects
				print_debug_range_bearing("Items", itemsRB)
				print_debug_range_bearing("Packing Station", packingStationRB)
				print_debug_range_bearing("Obstacles", obstaclesRB)
				print_debug_range_bearing("Row Markers", rowMarkerRB)
				print_debug_range_bearing("Shelves", shelfRB)
				print_debug_range_bearing("Picking Stations", pickingStationRB)
				print(warehouseBotSim._get_robot_position())
				print("=" * 50)

			# Update object positions (required for accurate detection)
			warehouseBotSim.UpdateObjectPositions()
			# # 1. Find the Goal (Picking Station 1)
			# # We check if the data for station 1 (at index 0) exists.
			# if pickingStationRB is not None and pickingStationRB[0] is not None:
			# 	# station1_data is a list: [range, bearing]
			# 	bearing_in_radians = pickingStationRB[0][1]
			# 	goal_deg = int(math.degrees(bearing_in_radians))
			# else:
			# 	goal_deg = None

			# # 2. Format Obstacle Data for NavClass
			# nav_obstacles = []
			# # Check if any obstacles were detected
			# if obstaclesRB is not None:
			# 	# Loop through each detected obstacle
			# 	for obs in obstaclesRB:
			# 		# obs is a list: [range, bearing]
			# 		dist_to_obs = obs[0] # Access range (distance) from index 0
			# 		bearing_rad = obs[1] # Access bearing from index 1
					
			# 		formatted_obs = Obstacle(
			# 			distance_to_robot=dist_to_obs,
			# 			degree=int(math.degrees(bearing_rad))
			# 		)
			# 		nav_obstacles.append(formatted_obs)

			# 3. Calculate Velocities and Set Robot Movement
			

			# if goal_deg is not None:
			# 	nav_results = navigation.calculate_goal_velocities(goal_deg, nav_obstacles)
			# 	forward_vel = nav_results['forward_vel']
			# 	rot_vel = nav_results['rotational_vel']
			# 	print(f"Goal Found at {goal_deg} deg. -> Fwd: {forward_vel:.2f} m/s, Rot: {rot_vel:.2f} rad/s")
			
			# else:
			# 	print("Searching for goal (Picking Station 1)...")
			# 	rot_vel = 0.15 # Spin slowly to find the goal

			# warehouseBotSim.SetTargetVelocities(forward_vel, rot_vel)
			warehouseBotSim.driveDistance(-0.5, 0.0)  # Rotate 15 degrees
			break
	

	except KeyboardInterrupt:
		print("\nStopping simulation...")
		warehouseBotSim.StopSimulator()
		print("Simulation stopped successfully. Goodbye!")

	except Exception as e:
		print(f"\nAn error occurred: {e}")
		print("Stopping simulation...")
		try:
			warehouseBotSim.StopSimulator()
		except:
			pass
		print("Please check your CoppeliaSim setup and try again.")
