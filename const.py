class parameter:
	def __init__(self):

		# ROBOT CONTROL PARAMETER
		self.distance_Kp = 100
		self.distance_Kd = 50
		self.degree_Kp = 1.0
		self.degree_Kd = 0.8
		self.MAX_PULSE = 1650
		self.MIN_PULSE = 1350
		self.PWM_OFFSET = 0

		# WAY POINT
		self.way_point_file = './way_point/hirako_target.csv'

		# MODE
		self.data_log_mode = True
		self.debug_mode = True
		self.wt_log_mode = False

		# CONTROL MODE
		self.control_mode = 2
		# 0:OMNICONTROL MODE, 1:DIAGNALCONTROL MODE, 2:FIXED HEAD CONTROL mode
		self.thruster_control = 0
		# 0:SIMPLE CONTROL, 1:PHASE CONTROL

		# CONTROL STRATEGY
		self.strategy = 1
		# 0:SIMPLE STRATEGY, 1:FLEX STRATEGY

		# OTHER
		self.main_target_distance_torelance = 3.0
		self.temp_target_distance_torelance = 1.0
		self.heading_torelance = 35.0
		self.duration = 10.0
		self.timer = 0.1
