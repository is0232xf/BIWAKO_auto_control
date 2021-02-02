
class Robot:
	def __init__(self, way_point):
		self.lon = 0.0
		self.lat = 0.0
		self.roll = 0.0
		self.pitch = 0.0
		self.yaw = 0.0
		self.way_point = way_point
		self.way_point_num = 0
		self.next_goal = way_point[self.way_point_num]
		self.pwm = 1500
		self.cmd = 0 # 0:Straight, 1:Turn right, 2:Turn left
		self.count = 0
		self.temp_goal = [0.0, 0.0]
        
	def update_next_goal(self):
		if self.way_point_num < len(self.way_point)-1:
			self.way_point_num = self.way_point_num + 1
			self.next_goal = self.way_point[self.way_point_num]
		else:
			self.way_point_num = -1
			print("Complete the mission!!!")
      
	def count_up(self):
   		self.count = self.count + 1
