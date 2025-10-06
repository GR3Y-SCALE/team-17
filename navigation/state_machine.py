from typing import Optional, Any
import time


class State:
    def __init__(self, robot: Any):
        # Shared robot/simulator context (e.g., COPPELIA_WarehouseRobot)
        self.robot = robot
        # Runs when state is instantiated
        print('current state: ', str(self))

    def run(self, event: Optional[str] = None) -> "State":
        return self

    def __repr__(self):
        return self.__str__()

    def __str__(self):
        return self.__class__.__name__


class StartState(State):
    def run(self, event: Optional[str] = None) -> "State":
        self.robot.SetTargetVelocities(0.5,0.0)
        if event == 'startup_done':
            return FindPickingStation(self.robot)
        # stay here until startup succeeds
        return self


class FindPickingStation(State):
    def run(self, event: Optional[str] = None) -> "State":
        print('Searching for picking station...')


        if event == 'found_picking_station':
            return MoveToShelf(self.robot)
        elif event == 'search_failed':
            # stay in this state or raise; here we stay to keep trying
            return self
        else:
            # no relevant event -> remain in the same state
            return self


class MoveToShelf(State):
    def run(self, event: Optional[str] = None) -> "State":
        print('moving to shelf...')
        # drive to shelf, path plan, etc.

        if event == 'arrived_at_shelf':
            return PlaceItemOnShelf(self.robot)
        elif event == 'navigation_blocked':
            # handle recovery; for now, keep trying
            return self
        else:
            return self


class PlaceItemOnShelf(State):
    def run(self, event: Optional[str] = None) -> "State":
        print('placing item on shelf...')
        # actuate gripper, verify placement, etc.

        if event == 'placed_item':
            return LeaveShelf(self.robot)
        elif event == 'placement_failed':
            # retry placement by staying in the same state
            return self
        else:
            return self


class LeaveShelf(State):
    def run(self, event: Optional[str] = None) -> "State":
        print('leaving shelf')
        # back out and clear the shelf area

        if event == 'left_shelf':
            return FindPickingStation(self.robot)
        elif event == 'exit_blocked':
            # try again
            return self
        else:
            return self


class StateMachine:
    def __init__(self, robot: Any):
        self.robot = robot
        self.state: State = StartState(robot)

    def update_state(self, event: Optional[str] = None) -> None:
        prev = self.state
        self.state = self.state.run(event)
        if prev is not self.state:
            print(f"transition: {prev} --({event})--> {self.state}")
