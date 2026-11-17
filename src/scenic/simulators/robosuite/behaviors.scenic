"""Behaviors for dynamic agents in robosuite."""

try:
    from scenic.simulators.robosuite.actions import *
except ModuleNotFoundError:
    pass    # ignore; error will be caught later if user attempts to run a simulation

behavior OpenGripper():
    take SetGripperAction(False)
    # Reference: OpenGripper (isaac/behaviors.scenic) repeats the action until
    # the fingers reach their target:
    #
    # behavior OpenGripper(threshold=0.002, maxSteps=100):
    #     sim = simulation()
    #     target = np.array(self.getGripperTargetPositions(True), dtype=float)
    #     try:
    #         do _RepeatAction(OpenGripperAction()) for maxSteps steps
    #     interrupt when _gripperReached(self, sim, target, threshold):
    #         abort

behavior CloseGripper():
    take SetGripperAction(True)
    # Reference: CloseGripper (isaac/behaviors.scenic):
    #
    # behavior CloseGripper(threshold=0.002, maxSteps=100):
    #     sim = simulation()
    #     target = np.array(self.getGripperTargetPositions(False), dtype=float)
    #     try:
    #         do _RepeatAction(CloseGripperAction()) for maxSteps steps
    #     interrupt when _gripperReached(self, sim, target, threshold):
    #         abort
