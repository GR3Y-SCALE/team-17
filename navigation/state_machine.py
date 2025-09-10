from typing import Optional
import time


class State:
    def __init__(self):
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
        # startup robot, home gripper, initialise motors cameras... etc.
        if event != 'startup_done':
            # If we haven't received the success event, fail fast (or return self to retry)
            raise RuntimeError('Robot failed to start')
        # On success, move to the next state
        return FindPickingStation()


class FindPickingStation(State):
    def run(self, event: Optional[str] = None) -> "State":
        print('Searching for picking station...')
        # turn robot, look, turn and so on until found. Some timeout condition if robot turns around full 360.

        if event == 'found_picking_station':
            return MoveToShelf()
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
            return PlaceItemOnShelf()
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
            return LeaveShelf()
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
            return FindPickingStation()
        elif event == 'exit_blocked':
            # try again
            return self
        else:
            return self


class StateMachine:
    def __init__(self):
        self.state: State = StartState()

    def update_state(self, event: Optional[str] = None) -> None:
        self.state = self.state.run(event)
