
import numpy as np
import cv2
import math


class Map_plot:
	def __init__(self, breadth, length, dim, clear, resol):
		self.breadth = breadth
		self.length = length
		self.dim = dim
		self.clear = clear
		self.resol = resol

		self.y_min = 0
		self.y_max = breadth
		self.x_min = 0
		self.x_max = length

		self.obs_1_x = 50
		self.obs_1_y = 100

		self.obs_2_x = 90
		self.obs_2_y = 50

		self.obs_3_x = 150
		self.obs_3_y = 100

		self.obs_4_x = 250
		self.obs_4_y = 80

		self.obs_5_x = 250
		self.obs_5_y = 90

		# self.map_image = []

	def plot_workspace(self):
		increase = self.dim + self.clear

		img = 255 * np.ones((self.breadth, self.length, 3), np.uint8)

		# Plot the circle
		cv2.circle(img, (50, self.breadth - 100), 3 + increase, (0, 0, 0), -1)
		cv2.circle(img, (50, self.breadth - 100), 3, (255, 0, 0), -1)
		cv2.circle(img, (90, self.breadth - 50), 3 + increase, (0, 0, 0), -1)
		cv2.circle(img, (90, self.breadth - 50), 3, (255, 0, 0), -1)
		cv2.circle(img, (150, self.breadth - 100), 3 + increase, (0, 0, 0), -1)
		cv2.circle(img, (150, self.breadth - 100), 3, (255, 0, 0), -1)
		cv2.circle(img, (250, self.breadth - 80), 3 + increase, (0, 0, 0), -1)
		cv2.circle(img, (250, self.breadth - 80), 3, (255, 0, 0), -1)
		cv2.circle(img, (250, self.breadth - 90), 3 + increase, (0, 0, 0), -1)
		cv2.circle(img, (250, self.breadth - 90), 3, (255, 0, 0), -1)


		res = cv2.resize(img, None, fx=self.resol, fy=self.resol, interpolation=cv2.INTER_CUBIC)

		self.map_image = res

		return res

	def check_obs(self, coordinate):

		increase = self.dim + self.clear
		point_x = coordinate[0]
		point_y = coordinate[1]

		dist_1 = np.sqrt((point_x - self.obs_1_x)**2 + (point_y - self.obs_1_y)**2)
		dist_2 = np.sqrt((point_x - self.obs_2_x)**2 + (point_y - self.obs_2_y)**2)
		dist_3 = np.sqrt((point_x - self.obs_3_x)**2 + (point_y - self.obs_3_y)**2)
		dist_4 = np.sqrt((point_x - self.obs_4_x)**2 + (point_y - self.obs_4_y)**2)
		dist_5 = np.sqrt((point_x - self.obs_5_x)**2 + (point_y - self.obs_5_y)**2)

		# print("  dist: ", dist_1)
		# print("  dist: ", dist_2)
		# print("  dist: ", dist_3)
		# print("  dist: ", dist_4)
		# print("  dist: ", dist_5)

		if dist_1 <= 3 + increase or dist_2 <= 3 + increase or dist_3 <= 3 + increase or dist_4 <= 3 + increase or dist_5 <= 3 + increase:
			return True
		else:
			return False



# work = Map_plot(150, 300, 0, 0, 4)
# map_image = work.plot_workspace()
# cv2.imshow('img', map_image)
# cv2.waitKey(0)
# print(work.check_obs([40, 40]))


