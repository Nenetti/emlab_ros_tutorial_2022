#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from flexbe_states.wait_state import WaitState
from move_base_state_tutorial.counter import CounterState
from move_base_state_tutorial.move_base_state import MoveBaseState
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Tue Jul 19 2022
@author: kanechika
'''
class move_base_tutorialSM(Behavior):
	'''
	description of move_base_tutorial
	'''


	def __init__(self):
		super(move_base_tutorialSM, self).__init__()
		self.name = 'move_base_tutorial'

		# parameters of this behavior

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:383 y:40, x:633 y:290
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])
		_state_machine.userdata.goal1 = [[2,3,0], [90]]
		_state_machine.userdata.goal2 = [[2,-3,0], [-90]]

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:329 y:124
			OperatableStateMachine.add('counter',
										CounterState(target_count=2),
										transitions={'over': 'finished', 'pass': 'move1'},
										autonomy={'over': Autonomy.Off, 'pass': Autonomy.Off})

			# x:579 y:124
			OperatableStateMachine.add('move1',
										MoveBaseState(action_topic="/move_base", timeout=15),
										transitions={'done': 'wait1', 'failed': 'failed'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'goal': 'goal1'})

			# x:829 y:424
			OperatableStateMachine.add('move2',
										MoveBaseState(action_topic="/move_base", timeout=15),
										transitions={'done': 'wait2', 'failed': 'failed'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'goal': 'goal2'})

			# x:857 y:124
			OperatableStateMachine.add('wait1',
										WaitState(wait_time=2),
										transitions={'done': 'move2'},
										autonomy={'done': Autonomy.Off})

			# x:357 y:424
			OperatableStateMachine.add('wait2',
										WaitState(wait_time=2),
										transitions={'done': 'counter'},
										autonomy={'done': Autonomy.Off})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
