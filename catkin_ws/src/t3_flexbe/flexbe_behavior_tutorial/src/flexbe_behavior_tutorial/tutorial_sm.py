#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from flexbe_state_tutorial.print_state import PrintState
from flexbe_state_tutorial.tutorial_check_state import TutorialInputState
from flexbe_states.wait_state import WaitState
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Tue Jul 19 2022
@author: kanechika
'''
class tutorialSM(Behavior):
	'''
	description of tutorial
	'''


	def __init__(self):
		super(tutorialSM, self).__init__()
		self.name = 'tutorial'

		# parameters of this behavior

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:983 y:140, x:130 y:365
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])
		_state_machine.userdata.input_str_data = "text"
		_state_machine.userdata.input_int_data = 1

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:142 y:67
			OperatableStateMachine.add('tutorrial_state',
										TutorialInputState(init_arg="init_text"),
										transitions={'route1': 'wait', 'route2': 'failed', 'ROUTE3': 'failed', 'ROUTE4': 'wait2'},
										autonomy={'route1': Autonomy.Off, 'route2': Autonomy.Off, 'ROUTE3': Autonomy.Off, 'ROUTE4': Autonomy.Off},
										remapping={'input_str_data': 'input_str_data', 'input_int_data': 'input_int_data', 'output_list': 'output_list', 'output_ndarray': 'output_ndarray'})

			# x:642 y:224
			OperatableStateMachine.add('print2',
										PrintState(),
										transitions={'done': 'finished'},
										autonomy={'done': Autonomy.Off},
										remapping={'print_data': 'output_ndarray'})

			# x:407 y:74
			OperatableStateMachine.add('wait',
										WaitState(wait_time=3),
										transitions={'done': 'print1'},
										autonomy={'done': Autonomy.Off})

			# x:407 y:224
			OperatableStateMachine.add('wait2',
										WaitState(wait_time=5),
										transitions={'done': 'print2'},
										autonomy={'done': Autonomy.Off})

			# x:642 y:74
			OperatableStateMachine.add('print1',
										PrintState(),
										transitions={'done': 'finished'},
										autonomy={'done': Autonomy.Off},
										remapping={'print_data': 'output_list'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
