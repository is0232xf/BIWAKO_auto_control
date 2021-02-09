class parameter:
	def __init__(self):

		# ROBOT CONTROL PARAMETER
		self.distance_Kp = 100
		self.distance_Kd = 50
		self.degree_Kp = 1.8
		self.degree_Kd = 0.8
		self.MAX_PULSE = 1662
		self.MIN_PULSE= 1362

		# WAY POINT
		self.way_point_file = './way_point/target.csv'

		# MODE
		self.data_log_mode = True
		self.debug_mode = False
		self.wt_log_mode = False

		#CONTROL MODE
		self.control_mode = 0
		# 0:OMNICONTROL MODE, 1:DIAGNALCONTROL MODE

		# OTHER
		self.main_target_distance_torelance = 3.0
		self.temp_target_distance_torelance = 1.0
		self.heading_torelance = 10.0
		self.duration = 10.0
