import time


class State:
    def __init__(self):
        # Runs when state is instantiated
        print('current state: ', str(self))

    def run(self, *args, **kwargs):
        pass

    def __repr__(self):
        return self.__str__()

    def __str__(self):
        return self.__class__.__name__


class StartState(State):
    """
    Run start-up procedures for nav. If startup does not complete,
    raise an exception (implements the TODO).
    """

    def __init__(self, event=None):
        super().__init__()
        self.event = event

    def run(self, event=None):
        if event is not None:
            self.event = event
        # startup robot, home gripper, initialise motors cameras...
        if self.event != 'startup_done':
            raise RuntimeError('Robot failed to start')

        # Otherwise, continue normally (no-op here)
        return 'startup_done'
    
class findPickingStation(State):
    def run(self):
        print('Searching for picking station...')

        # turn robot, look, turn and so on until found. Some timeout condition if robot turns around full 360.

        if event == 'found_picking_station':
            return collectFromPickingStation()
        else:
            # report unsuccessgful search

class moveToShelf(State):
    def run(self):
        print('moving to shelf...')

        # move to shelf

        if event == 'arrived_at_shelf':
            return placeItemOnShelf()
        elif event == '...':
            # extra logic to handle not arriving at the shelf

class leaveShelf(State):
    def run(self):
        print('leaving shelf')
        # leave shelf

        if event == 'left_shelf':
            return findPickingStation()
        elif event == '...':
            # keep trying to leave



class stateMachine():
    def __init__(self):
        self.state = StartState()

    def update_state(self):
        self.state = self.state.run()

