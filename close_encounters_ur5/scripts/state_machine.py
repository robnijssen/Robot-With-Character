#!/usr/bin/env python
import rospy

"""
Blueprints for the state machines and states used in the node's python files.
"""

# state machine blueprint

class StateMachineBlueprint:
    def __init__(self, initialState):
        self.currentState = initialState
    def runAll(self):
        while not rospy.is_shutdown():
            # overwrite nextState
            self.nextState = self.currentState.next()
            if self.currentState != self.nextState:
                # transition occurred. trainsition must run and currectState must be overwritten
                self.nextState.transitionRun()
                self.currentState = self.nextState
            self.currentState.mainRun()

# state blueprint

class StateBlueprint:
    def transitionRun(self):
        assert 0, "State transitionRun not implemented"
    def mainRun(self):
        assert 0, "State mainRun not implemented"
    def next(self):
        assert 0, "State next not implemented"