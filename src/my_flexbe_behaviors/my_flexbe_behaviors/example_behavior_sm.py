from flexbe_core import Behavior, OperatableStateMachine
from my_flexbe_behaviors.states.talk_state import TalkState

class ExampleBehaviorSM(Behavior):
    def __init__(self):
        super().__init__()
        self.name = 'ExampleBehavior'

    def create(self):
        with OperatableStateMachine(outcomes=['finished']) as sm:
            OperatableStateMachine.add('Talk',
                                       TalkState(),
                                       transitions={'done': 'finished'})
        return sm
