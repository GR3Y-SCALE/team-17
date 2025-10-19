#!/usr/bin/env python3

import os,sys,time,math,csv
from enum import Enum, auto

from order_processing.process_order import Process_Order

from mobility.drive_system import DriveSystem
from navigation.NavClass import NavClass
from vision.cam_goodge_again import VisionSystem
from grippy.grippy_i2c import GrippyController


critical_fault = False

process_order = Process_Order()

# try:
#     robot = DriveSystem()
#     pass
# except Exception as e:
#     print(f"[ ERROR ]: Cannot initialise mobility system. Reason: {e}", file=sys.stderr)
#     critical_fault = True
robot = DriveSystem()

try:
    vision = VisionSystem()
except Exception as e:
    print(f"[ ERROR ]: Cannot initialise vision system. Reason: {e}", file=sys.stderr)
    critical_fault = True

try:
    gripper = GrippyController()
except:
    print(f"[ ERROR ]: Cannot initialise item collection system. Reason: {e}", file=sys.stderr)
    critical_fault = True

try:
    nav = NavClass(FOV=140, width=0.16, range_finder=True)
except Exception as e:
    print(f"[ ERROR ]: Cannot initialise navigation system. Reason: {e}", file=sys.stderr)
    critical_fault = True
# nav = NavClass(FOV=140, width=0.16)

if not critical_fault: 
    print("[ OK ] Systems initialised, ready to rock and roll")
    gripper.led_green()
else:
    print("[ RUH ROH ] somethings borked")
    gripper.led_red()



class robot_state(Enum):
    DEBUGGING = auto()
    APPROACH_PICKING_STATION = auto()
    COLLECT_ITEM = auto()
    ENTER_ISLE = auto()
    DESPOSIT_ITEM = auto()
    RETURN_TO_PICKING_STATION = auto()

# class collection_state(Enum):
#     ENTER_RAMP_BAY = auto()
#     APPROACHING_ITEM = auto()
#     PICKING_UP_ITEM = auto()
# robot_collection_state = collection_state.HEADING_TO_CORRECT_BAY


def go_to_landmark(object_lambda, distance, speed, debug=True):
    '''
    Drives the robot to a landmark in the warehouse using potential field, handles the event of object not being visible.

    Args:
        pass lambda function of getter for vision object
        distance: distance from the object the robot will stop at
        speed: forward velocity of robot when going to landmark.
    '''

    # To Mozz
    # Switch case for different landmark types
    # to methods, it could be in the VisionSystem class itself
    # or you could have it as a parameter to this function.
    # Love Dan
    vision.UpdateObjects()
    landmark = object_lambda()
    range_finder_distance = None

    if landmark is None:
        if debug: print("Landmark not visible")
        while landmark is None:
            if debug: print("Searching...")
            robot.set_target_velocities(0.0, -0.05)

            vision.UpdateObjects()
            landmark = object_lambda()
            time.sleep(0.001)
        time.sleep(0.1)
    time.sleep(0.15)
    robot.set_target_velocities(0.0, 0.0)
    if debug: print("Found!")
    distance_to_landmark = landmark[0]
    print("Distance to landmark: " + str(round(distance_to_landmark,2)))
    while distance_to_landmark > distance:
        if debug: print("Going to landmark...")
        vision.UpdateObjects()
        landmark = object_lambda()

        if range_finder_distance is None:
            distance_to_landmark = landmark[0]
        else:
            distance_to_landmark = nav.get_range_finder_distance() / 1000.0 # convert to

        # Add some functionality to use range finder when error is low

        nav_data = nav.calculate_goal_velocities(landmark[1], None, False)
        robot.set_target_velocities(speed, -nav_data['rotational_vel'])
        print("Angle error:" + str(nav_data['rotational_vel']))
        time.sleep(0.01)
        if debug: print('Distance to landmark: ' + str(round(landmark[0],2)))
        if nav_data['rotational_vel'] < 10:
            range_finder_distance = nav.get_range_finder_distance() / 1000 # convert to meters
        else:
            range_finder_distance = None # disable range finder when error is too high
    time.sleep(0.15)
    robot.set_target_velocities(0.0, 0.0)
    print("Complete!")

def center_to_landmark(object_lambda, target_error, speed, debug=False):
    '''
    Centres robot to landmark without driving forward, assumes landmark is visible.
    '''
    vision.UpdateObjects()
    landmark = object_lambda()

    nav_data = nav.calculate_goal_velocities(landmark[1], None, False)
    while nav_data['rotational_vel'] > target_error:
        if debug: print("Error" + str(nav_data['rotational_vel']))

        vision.UpdateObjects()
        landmark = object_lambda()
        nav_data = nav.calculate_goal_velocities(landmark[1], None, False)
        robot.set_target_velocities(0.0, -nav_data['rotational_vel'])
        time.sleep(0.001)
    time.sleep(0.15)
    robot.set_target_velocities(0.0, 0.0)
    if debug: print("Centred!")

# def drive_by_range(target_distance, speed, debug=False):
#     target_distance = target_distance * 1000 # convert to mm

#     if speed > 0:
#         while nav.get_range_finder_distance() > target_distance:
#             if debug: print("Distance to face: " + str(nav.get_range_finder_distance()))
#             robot.set_target_velocities(speed,0.0)
#             time.sleep(0.001)
#     else:
#         while nav.get_range_finder_distance() < target_distance:
#             if debug: print("Distance to face: " + str(nav.get_range_finder_distance()))
#             robot.set_target_velocities(speed,0.0)
#             time.sleep(0.001)
#     robot.set_target_velocities(0.0,0.0)
#     print("Reached set distance: " + str(nav.get_range_finder_distance()))

def drive_by_range(target_distance, speed, padding_m=0.015, debug=False):
    target_distance_mm = target_distance * 1000  # convert to mm
    padding_mm = padding_m * 1000

    while True:
        current_distance_mm = nav.get_range_finder_distance()

        if debug:
            print(f"Current distance: {current_distance_mm}mm, Target distance: {target_distance_mm}mm")

        # Check if we are within the target range with padding
        if abs(current_distance_mm - target_distance_mm) <= padding_mm:
            robot.set_target_velocities(0.0, 0.0)
            print(f"Reached set distance (within {padding_mm}mm padding): {current_distance_mm}mm")
            break

        # Move forward if too far
        if current_distance_mm > target_distance_mm + padding_mm:
            robot.set_target_velocities(abs(speed), 0.0)
        # Move backward if too close
        elif current_distance_mm < target_distance_mm - padding_mm:
            robot.set_target_velocities(-abs(speed), 0.0)
        
        time.sleep(0.01)
    time.sleep(0.15)
    robot.set_target_velocities(0.0, 0.0)


iteration = 0
dt = 0.1

(item_num, 
picking_station_num, 
shelf_num, 
bay_num, 
bay_height, 
item_name) = process_order.import_order(debug=True) # Process order

# Real world measurements
bay_width = 0.265 # Meters
# range finder height = 85mm
gripper_to_range_finder = 0.140 # from tip range finder



# picking_station_order = [0, 1, 2]
# shelf_number = [0, 3, 5]
# bay_number = [0, 1, 0]
# bay_height = [0, 1, 2]
# shelf_number_for_deposit = [0,3,4]
# # turn_padding = [11, -40, -50]
bay_depth = [0.05, 0.05, 0.05] # How far in the robot needs to go for each shelf, partly redundant
# lift_position = [110, 140, 0]
lift_position_collection = [110,140,0] # Optimal lift positions for collecting items
lift_position_bay = [120, 70, 30] # Tweak these for item placement in the shelf


def main():
    # last_time = time.time()
    iteration = 0
    robot_navigation_state = robot_state.ENTER_ISLE
    try:
        while True:
            # now = time.time()
            # elapsed = now - last_time
            # while elapsed < dt:
            #     time.sleep(dt - elapsed)
            #     continue
            # last_time = now 
            # Update camera objects
            match robot_navigation_state:
                case robot_state.DEBUGGING:
                    vision.UpdateObjects()
                    drive_by_range(0.2, 0.1)
                    break
                    # robot.turn_degrees(90)

                case robot_state.APPROACH_PICKING_STATION:
                    gripper.led_yellow()
                    vision.UpdateObjects()
                    # Use first shelf to navigate to picking station and turn to face the correct marker
                    robot_navigation_state = robot_state.COLLECT_ITEM
                case robot_state.COLLECT_ITEM:
                    # collection_item_index = picking_station_num[iteration] - 1 # Picking station's index starts at 1
                    collection_item_index = 0

                    gripper.led_yellow()
                    print("Entering ramp")
                    vision.UpdateObjects()
                    print(vision.get_all())

                    # drive up ramp , might need gain adjustment for each different ramp
                    gripper.gripper_open()
                    gripper.lift(lift_position_collection[0])
                    time.sleep(1.5)
                    go_to_landmark(lambda : vision.get_items()[collection_item_index], 0.15, 0.3, False)
                    gripper.lift(lift_position_collection[1])
                    time.sleep(1)
                    
                    # Nudge closer to item to ensure it is inside the gripper
                    drive_by_range(0.185, 0.2)
                    gripper.gripper_open()
                    time.sleep(1)

                    # Center to item if needed
                    center_to_landmark(lambda : vision.get_items()[collection_item_index], 0.01, 20, True)
                    gripper.gripper_close()
                    print("Item Collected")
                    gripper.lift(lift_position_collection[2])

                    # back out of ramp
                    drive_by_range(0.6, 0.2)
                    time.sleep(1) # makes sure everthing has settled
                    robot_navigation_state = robot_state.ENTER_ISLE
                case robot_state.ENTER_ISLE:
                    gripper.led_yellow()

                    if shelf_num[iteration] <= 1: # The first and second shelf, using closest wall as distance reference and therefore needing to turn right from ramp
                        robot.turn_degrees(85)
                        time.sleep(0.5)
                        drive_by_range(0.30, 0.2) # optimal stopping distance to enter the first isle
                        print("At first isle")
                        turn_direction = 1
                    elif shelf_num[iteration] <= 3: # This is the second isle and therefore needs a 
                        robot.turn_degrees(90)
                        time.sleep(0.5)
                        drive_by_range(0.97, 0.1) # optimal stopping distance to enter the second isle
                        print("At second isle")
                        turn_direction = 1
                    else:
                        robot.turn_degrees(-90)
                        time.sleep(0.5)
                        drive_by_range(0.3, 0.1) # optimal stopping distance to enter third isle
                        print("At third isle")
                        turn_direction = -1
                        # Note I might have to stop by the second isle to align myself better if turning acurracy is poor

                    # The robot has now moved just in front of the isle, now it needs to face the row marker.
                    time.sleep(0.15)
                    robot.turn_degrees(90 * turn_direction)
                    time.sleep(1)

                    vision.UpdateObjects()
                    # Now drive into the isle at the correct distance from the row marker to align correctly to bay
                    stopping_distance = bay_width * (4 - bay_num[iteration]) # third bay is closest to the row marker, invert
                    go_to_landmark(lambda: vision.get_row_markers()[0], stopping_distance - gripper_to_range_finder, 0.15)
                    print("At correct bay position")


                    turn_direction = 1 if (shelf_num[iteration] % 2) else -1 # Turn 90 degrees to the left if even
                    robot.turn_degrees(90 * turn_direction)
                    # Robot is now facing the correct bay, supposedly
                    print("Positioned at isle, getting ready to place item.")
                    time.sleep(1)
                    robot_navigation_state = robot_state.DESPOSIT_ITEM
                case robot_state.DESPOSIT_ITEM:
                    starting_distance = nav.get_range_finder_distance() / 1000 # Convert to meters
                    print("Distance to bay from robot " + str(starting_distance))

                    gripper.led_red()
                    distance_buffer = gripper_to_range_finder + 0.2 # buffer so gripper doesnt hit bay
                    if starting_distance < distance_buffer:
                        print("Too close to bay, backing up a bit")
                        drive_by_range(distance_buffer, 0.1)
                        starting_distance = nav.get_range_finder_distance() / 1000 # Convert to meters
                        print("New distance to bay from robot " + str(starting_distance))
                    gripper.lift(lift_position_bay[bay_height[iteration]])

                    print("Raised lift to correct height")
                    time.sleep(1)
                    print("Now driving into bay.")
                    drive_by_range(gripper_to_range_finder + 0.025, 0.15)

                    gripper.gripper_open()
                    drive_by_range(starting_distance, 0.1)
                    print("Item Deposited!!!")
                    robot_navigation_state = robot_state.RETURN_TO_PICKING_STATION
                case robot_state.RETURN_TO_PICKING_STATION:
                    # update camera
                    # turn to find row marker, then turn 180 degrees from it
                    # leave shelf area
                    # search from left to right to find picking station
                    # use potential field to drive to picking station, possibly modifying the field so it drives alongside it
                    # once at picking station, turn until bay marker is visible
                    iteration += 1
                    print(f"Completed {iteration} full iterations")
                    robot_navigation_state = robot_state.COLLECT_ITEM

            
    except KeyboardInterrupt:
        print("Exiting")
        gripper.led_off()
        vision.camera_release()
        robot.stop_all()



if __name__ == "__main__":
    main()
