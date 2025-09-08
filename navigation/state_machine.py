import time

class State:
    def __init__(self):
        # Runs when state is instantiated
        print('current state: ',str(self))

    def run(self):
        pass

    def __repr__(self):
        return self.__str__()
    
    def __str__(self):
        return self.__class__.__name__
    
    class startState(State):
        # run start up procedures for nav stuff

        event = 'startup_done'

        try event == 'startup_done':

            except:
            #TODO: rasise exception for robot failing to start
                raise Exception
         