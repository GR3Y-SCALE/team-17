#!/usr/bin/python

# Import the warehouse bot library
from warehousebot_lib import *

# Import additional modules
import os,sys,time
from collections import namedtuple
from enum import Enum, auto
# my stuff
sys.path.append(os.path.join(os.path.dirname(__file__), '..'))
from navigation.NavClass import NavClass
from navigation.state_machine import StateMachine

Obstacle = namedtuple('Obstacle', ['distance_to_robot', 'degree'])

def clear_screen():
	"""Clear the terminal screen for better output readability"""
	os.system('cls' if os.name == 'nt' else 'clear')



# CONFIGURE SCENE PARAMETERS
sceneParameters = SceneParameters()

# Set which items appear at which picking stations (0=station 1, 1=station 2, 2=station 3)
# Set to -1 to leave a station empty
sceneParameters.pickingStationContents[0] = warehouseObjects.bowl    # Bowl at picking station 1
sceneParameters.pickingStationContents[1] = warehouseObjects.mug     # Mug at picking station 2  
sceneParameters.pickingStationContents[2] = warehouseObjects.bottle  # Bottle at picking station 3

# Set obstacle starting positions [x, y] in meters
# Use -1 to keep current CoppeliaSim position, None to disable obstacle
sceneParameters.obstacle0_StartingPosition = [-1, -1]
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
robotParameters.maxRowMarkerDetectionDistance = 3.0    # Row markers

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
		# Initialise state machine with the simulator and pass it into NavClass
		sm = StateMachine(warehouseBotSim)
		sm.update_state('startup_done')
		navigation = NavClass(140, state_machine=sm)

		# Main control loop
		print("Starting main control loop...")
		print("Robot is now ready for navigation commands.")
		
		goal_deg = None
		warehouseBotSim.SetTargetVelocities(0.0, 0.0)  # Stop the robot
		

		forward_vel = 0.0
		rot_vel = 0.0

		

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
		time.sleep(1)
		warehouseBotSim.UpdateObjectPositions()
		if shelfRB[5] is None:
			# Cannot see the 5th shelf so turn to find it
			print("Searching for fifth shelf...")
			while shelfRB[5] is None:
				warehouseBotSim.SetTargetVelocities(0.0, 0.5)
				warehouseBotSim.UpdateObjectPositions()
				_, _, _, _, shelfRB, _ = warehouseBotSim.GetDetectedObjects([warehouseObjects.shelves])
				time.sleep(0.01)
			warehouseBotSim.SetTargetVelocities(0.0, 0.0)
			print("Found! Going to waypoint...")
		while shelfRB[5][0] > 0.8:
			warehouseBotSim.UpdateObjectPositions()
			_, _, _, _, shelfRB, _ = warehouseBotSim.GetDetectedObjects([warehouseObjects.shelves])
			nav_data = navigation.calculate_goal_velocities(int(math.degrees(shelfRB[5][1])))
			warehouseBotSim.SetTargetVelocities(0.02, -nav_data['rotational_vel'])
			time.sleep(0.001)
		warehouseBotSim.SetTargetVelocities(0.0,0.0)
		print("Stopping at 50CM before shelf")
		time.sleep(1)


		# localise to known position, the third row marker
		if rowMarkerRB[2] is None:
			print("Searching for third row marker...")
			warehouseBotSim.SetTargetVelocities(0.0, 0.2)
			while rowMarkerRB[2] is None:
				print_debug_range_bearing("Row Markers", rowMarkerRB)
				warehouseBotSim.UpdateObjectPositions()
				_, _, _, rowMarkerRB, _, _ = warehouseBotSim.GetDetectedObjects([warehouseObjects.row_markers])
				time.sleep(0.01)
			print("Found! Entering...") 
			warehouseBotSim.SetTargetVelocities(0.0, 0.0)


		warehouseBotSim.UpdateObjectPositions()
		marker_heading = rowMarkerRB[2][1]
		nav_data = navigation.calculate_goal_velocities(int(math.degrees(marker_heading)))
		warehouseBotSim.SetTargetVelocities(0.05, -nav_data['rotational_vel'])
		while rowMarkerRB[2][0] > 1: # 1M
			warehouseBotSim.UpdateObjectPositions()
			_, _, _, rowMarkerRB, _, _ = warehouseBotSim.GetDetectedObjects([warehouseObjects.row_markers])
			marker_heading = rowMarkerRB[2][1]
			nav_data = navigation.calculate_goal_velocities(int(math.degrees(marker_heading)))
			warehouseBotSim.SetTargetVelocities(0.02, -nav_data['rotational_vel'])
			time.sleep(0.001)
		warehouseBotSim.SetTargetVelocities(0.0,0.0)
		print("Stopping at 10CM before marker")
		time.sleep(1)
		
		warehouseBotSim.UpdateObjectPositions()
		_, _, _, _, _, pickingStationRB = warehouseBotSim.GetDetectedObjects([warehouseObjects.PickingStationMarkers])
		# Known distance from third marker, now reverse:
		warehouseBotSim.driveDistance(-0.18, 0.0, 0.0)
		time.sleep(1)
		warehouseBotSim.driveDistance(0.0, 0.0, 90.0)
		time.sleep(1)
		warehouseBotSim.driveDistance(0.0, 0.8, 0.0) # best is 0.8
		time.sleep(1)
		warehouseBotSim.SetTargetVelocities(0.0, 0.0)

		# if pickingStationRB[0] is None:
		# 	warehouseBotSim.SetTargetVelocities(0.0,0.2)
		# 	while pickingStationRB[0] is None:
		# 		warehouseBotSim.UpdateObjectPositions()
		# 		_,_,_,_,_,pickingStationRB = warehouseBotSim.GetDetectedObjects([warehouseObjects.PickingStationMarkers])
		# 		time.sleep(0.01)
		# 	warehouseBotSim.SetTargetVelocities(0.0,0.0)

		# warehouseBotSim.SetTargetVelocities(0.01,0.0)
		# while pickingStationRB[0][0] > 0.3:
		# 	warehouseBotSim.UpdateObjectPositions()
		# 	_,_,_,_,_,pickingStationRB = warehouseBotSim.GetDetectedObjects([warehouseObjects.PickingStationMarkers])
		# 	nav_data = navigation.calculate_goal_velocities(int(math.degrees(pickingStationRB[0][1])), None, False)
		# 	warehouseBotSim.SetTargetVelocities(0.005, -nav_data['rotational_vel'])
		# 	time.sleep(0.01)
		# warehouseBotSim.SetTargetVelocities(0.0,0.0)
		# if shelfRB[2] is None:
		# 	warehouseBotSim.SetTargetVelocities(0.0,-0.2)
		# 	while shelfRB[2] is None:
		# 		print(shelfRB[2])
		# 		warehouseBotSim.UpdateObjectPositions()
		# 		_,_,_,_,shelfRB,_ = warehouseBotSim.GetDetectedObjects([warehouseObjects.PickingStationMarkers])
		# 		time.sleep(0.01)
		# warehouseBotSim.SetTargetVelocities(0.0,0.0)

		# warehouseBotSim.SetTargetVelocities(0.01,0.0)
		# while shelfRB[2][0] > 0.5:
		# 	warehouseBotSim.UpdateObjectPositions()
		# 	_,_,_,_,shelfRB,_ = warehouseBotSim.GetDetectedObjects([warehouseObjects.PickingStationMarkers])
		# 	nav_data = navigation.calculate_goal_velocities(int(math.degrees(shelfRB[2][0])), None, False)
		# 	warehouseBotSim.SetTargetVelocities(0.005, -nav_data['rotational_vel'])
		# 	time.sleep(0.01)
		# warehouseBotSim.SetTargetVelocities(0.0,0.0)




		# now look for picking station 1
		while pickingStationRB[0] is None:
			warehouseBotSim.SetTargetVelocities(0.0, 0.3)
			warehouseBotSim.UpdateObjectPositions()
			_, _, _, _, _, pickingStationRB = warehouseBotSim.GetDetectedObjects([warehouseObjects.PickingStationMarkers])
			time.sleep(0.01)
		warehouseBotSim.SetTargetVelocities(0.0, 0.0)

		# now at picking station 1, here comes the meat and potatoes
		class AwesomeSM(Enum):
			COLLECT_ITEM = auto()
			FIND_SHELF = auto()
			DEPOSIT_ITEM = auto()
			RETURN_TO_PICKING_STATION = auto()
		state = AwesomeSM.COLLECT_ITEM
		item_in_gripper = False
		depositing = False

		# order to visit picking stations (use list, not set; sets are unordered/non-indexable)
		picking_station_number = [0, 1, 2]
		shelf_number = [0, 3, 5]
		bay_number = [0, 1, 0]
		bay_height = [0, 1, 2]
		shelf_number_for_deposit = [0,3,4]
		turn_padding = [11, -40, -50]
		bay_depth = [0.05, 0.05, 0.05]
		iteration = 0
		dt = 0.1  # desired step size (50 ms)
		last_time = time.time()
		CAMERA_FOV = 140  # degrees, must match NavClass FOV

		while True:
			now = time.time()
			elapsed = now - last_time
			if elapsed < dt:
				time.sleep(dt - elapsed)
				continue
			last_time = now 


			match state:
				case AwesomeSM.COLLECT_ITEM:
					if item_in_gripper == False:
						print('Aligning myself with the picking bay...')
						warehouseBotSim.UpdateObjectPositions()
						_, _, _, _, _, pickingStationRB = warehouseBotSim.GetDetectedObjects([warehouseObjects.PickingStationMarkers])
						chosen_station = pickingStationRB[picking_station_number[iteration]]
						if chosen_station is None:
							print("A1")
							while chosen_station is None:
								print("A2")
								warehouseBotSim.SetTargetVelocities(0.0, -0.1)
								warehouseBotSim.UpdateObjectPositions()
								_, _, _, _, _, pickingStationRB = warehouseBotSim.GetDetectedObjects([warehouseObjects.PickingStationMarkers])
								chosen_station = pickingStationRB[picking_station_number[iteration]]
						warehouseBotSim.SetTargetVelocities(0.0, 0.0)
						while chosen_station[0] > 0.1 and chosen_station[0] is not None:
							print("A3")
							chosen_station = pickingStationRB[picking_station_number[iteration]]
							warehouseBotSim.UpdateObjectPositions()
							_, _, _, _, _, pickingStationRB = warehouseBotSim.GetDetectedObjects([warehouseObjects.PickingStationMarkers])
							nav_data = navigation.calculate_goal_velocities(int(math.degrees(chosen_station[1])), None, True)
							warehouseBotSim.SetTargetVelocities(0.02, -nav_data['rotational_vel'])
							time.sleep(0.001)
							print('Honing in... distance: ' + str(round(chosen_station[0],2)))
						warehouseBotSim.SetTargetVelocities(0.0, 0.0)
						print('Stopping at 15CM before picking station')
						time.sleep(1)
						print('Collecting item...')
						if warehouseBotSim.CollectItem() is not None:
							print('Item collected successfully!')
							item_in_gripper = True
						else:
							raise Exception('Item collection failed, aborting!')
					else:
						time.sleep(1)
						print('going to shelf...')
						state = AwesomeSM.FIND_SHELF

				case AwesomeSM.FIND_SHELF:
					print("Finding shelf...")
					warehouseBotSim.UpdateObjectPositions()
					_, _, _, _, shelfRB, _ = warehouseBotSim.GetDetectedObjects([warehouseObjects.shelves])

					while pickingStationRB[iteration][0] < 0.35: # leave picking station
						warehouseBotSim.UpdateObjectPositions()
						_, _, _, _, _, pickingStationRB = warehouseBotSim.GetDetectedObjects([warehouseObjects.PickingStationMarkers])
						nav_data = navigation.calculate_goal_velocities(int(math.degrees(pickingStationRB[iteration][1])), None, True)
						warehouseBotSim.SetTargetVelocities(-0.01, -nav_data['rotational_vel'])
						time.sleep(0.005)
					warehouseBotSim.SetTargetVelocities(0.0,0.0)

					chosen_shelf = shelfRB[shelf_number[iteration]]
					while chosen_shelf is None:
						warehouseBotSim.SetTargetVelocities(0.0, -0.2)
						warehouseBotSim.UpdateObjectPositions()
						_, _, _, _, shelfRB, _ = warehouseBotSim.GetDetectedObjects([warehouseObjects.shelves])
						chosen_shelf = shelfRB[shelf_number[iteration]]
						time.sleep(0.005)
					warehouseBotSim.SetTargetVelocities(0.0, 0.0)
					print("Heading to shelf...")
					while chosen_shelf[0] > 0.5 and chosen_shelf is not None:
						chosen_shelf = shelfRB[shelf_number[iteration]]
						warehouseBotSim.UpdateObjectPositions()
						_, _, _, _, shelfRB, _ = warehouseBotSim.GetDetectedObjects([warehouseObjects.shelves])
						nav_data = navigation.calculate_goal_velocities(int(math.degrees(chosen_shelf[1])) + turn_padding[iteration], None, True)
						warehouseBotSim.SetTargetVelocities(0.01, -nav_data['rotational_vel'])
						time.sleep(0.001)
						print('Honing in... distance: ' + str(chosen_shelf[0]))
					warehouseBotSim.SetTargetVelocities(0.0, 0.0)
					print("Stopping at 10CM before shelf")
					time.sleep(1)
					print("Going to marker to line up with shelf...")
					warehouseBotSim.UpdateObjectPositions
					_, _, _, rowMarkerRB, _, _ = warehouseBotSim.GetDetectedObjects([warehouseObjects.row_markers])
					time.sleep(0.01)
					if rowMarkerRB[iteration] is None:
						print("Searching for row marker...")
						warehouseBotSim.SetTargetVelocities(0.0, -0.2)
						while rowMarkerRB[iteration] is None:
							warehouseBotSim.UpdateObjectPositions()
							_, _, _, rowMarkerRB, _, _ = warehouseBotSim.GetDetectedObjects([warehouseObjects.row_markers])
							time.sleep(0.01)
					warehouseBotSim.SetTargetVelocities(0.0, 0.0)

					while rowMarkerRB is not None and rowMarkerRB[iteration][0] > 0.08:
						print("Honing in... distance: " + str(rowMarkerRB[iteration][0]))
						warehouseBotSim.UpdateObjectPositions()
						_, _, _, rowMarkerRB, _, _ = warehouseBotSim.GetDetectedObjects([warehouseObjects.row_markers])
						nav_data = navigation.calculate_goal_velocities(int(math.degrees(rowMarkerRB[iteration][1])), None, True)
						warehouseBotSim.SetTargetVelocities(0.01, -nav_data["rotational_vel"])
						time.sleep(0.001)
					warehouseBotSim.SetTargetVelocities(0.0,0.0)
					warehouseBotSim.UpdateObjectPositions()
					_, _, _, rowMarkerRB, _, _ = warehouseBotSim.GetDetectedObjects([warehouseObjects.row_markers])
					state = AwesomeSM.DEPOSIT_ITEM
					#shelf is 8.52cm * 25.1cm height * width

				case AwesomeSM.DEPOSIT_ITEM:
					print("Depositing item...")
					distance_to_bay = round((0.251 * (3 - bay_number[iteration]) ) + rowMarkerRB[iteration][0],2)
					while rowMarkerRB[iteration][0] < distance_to_bay:
						warehouseBotSim.UpdateObjectPositions()
						_, _, _, rowMarkerRB, _, _ = warehouseBotSim.GetDetectedObjects([warehouseObjects.row_markers])
						nav_data = navigation.calculate_goal_velocities(int(math.degrees(rowMarkerRB[iteration][1])), None, True)
						warehouseBotSim.SetTargetVelocities(-0.01, -nav_data["rotational_vel"])
						time.sleep(0.005)
					print("At bay, now turning...")
					warehouseBotSim.SetTargetVelocities(0.0,0.0)
					time.sleep(1)
					direction = 90 if not shelf_number_for_deposit[iteration] % 2 else -90
					warehouseBotSim.driveDistance(0.0,0.0,direction)
					time.sleep(1)
					warehouseBotSim.driveDistance(0.0,bay_depth[iteration],0.0)
					# warehouseBotSim.UpdateObjectPositions()
					# _, _, _, _, shelfRB, _ = warehouseBotSim.GetDetectedObjects([warehouseObjects.shelves])
					# warehouseBotSim.SetTargetVelocities(0.01,0.0)
					# while shelfRB[shelf_number_for_deposit[iteration]][0] > 0.4: # Distance to shelf
					# 	warehouseBotSim.UpdateObjectPositions()
					# 	_, _, _, _, shelfRB, _ = warehouseBotSim.GetDetectedObjects([warehouseObjects.shelves])
					# 	time.sleep(0.01)
					warehouseBotSim.SetTargetVelocities(0.0,0.0)
					time.sleep(0.5)
					warehouseBotSim.DropItemInClosestShelfBay()
					item_in_gripper = False
					time.sleep(0.1)
					warehouseBotSim.SetTargetVelocities(-0.01,0.0)
					while shelfRB[shelf_number_for_deposit[iteration]][0] > 0.5: # Distance to shelf
						warehouseBotSim.UpdateObjectPositions()
						_, _, _, _, shelfRB, _ = warehouseBotSim.GetDetectedObjects([warehouseObjects.shelves])
						time.sleep(0.01)

					warehouseBotSim.driveDistance(0.0,-bay_depth[iteration],0.0)
					state = AwesomeSM.RETURN_TO_PICKING_STATION
				
				case AwesomeSM.RETURN_TO_PICKING_STATION:
					print("Returning to picking station...")
					warehouseBotSim.UpdateObjectPositions()
					_, _, _, rowMarkerRB, _, _ = warehouseBotSim.GetDetectedObjects([warehouseObjects.row_markers])
					if rowMarkerRB[iteration] is None:
						while rowMarkerRB[iteration] is None:
							warehouseBotSim.UpdateObjectPositions()
							_, _, _, rowMarkerRB, _, _ = warehouseBotSim.GetDetectedObjects([warehouseObjects.row_markers])
							warehouseBotSim.SetTargetVelocities(0.0,-0.5)
						warehouseBotSim.SetTargetVelocities(0.0,0.0)
					
					while rowMarkerRB[iteration][0] < 1:
						warehouseBotSim.UpdateObjectPositions()
						_, _, _, rowMarkerRB, _, _ = warehouseBotSim.GetDetectedObjects([warehouseObjects.row_markers])
						nav_data = navigation.calculate_goal_velocities(int(math.degrees(rowMarkerRB[iteration][1])), None, True)
						warehouseBotSim.SetTargetVelocities(-0.01, -nav_data["rotational_vel"])
					warehouseBotSim.SetTargetVelocities(0.0,0.0)
					time.sleep(1)
					# warehouseBotSim.driveDistance(0.0,0.0,180) # Turn around to see the picking station
					# Robot has now left the shelf

					if iteration == 0:
						iteration += iteration
						state = AwesomeSM.COLLECT_ITEM


					
						
						

					print(f"Incrementing station counter: {picking_station_number[iteration]} -> {picking_station_number[(iteration + 1) % len(picking_station_number)]}")
					iteration = (iteration + 1) % len(picking_station_number)


	

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
