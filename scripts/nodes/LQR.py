import numpy as np
import math
class LQR():
	def __init__(self,):
		self.yaw=0
		self.state=np.zeros((12,1))
		self.K=np.zeros((4,12))
		Kraw='''2 1 -0.885
5 1 -0.739
7 1 1.193
10 1 0.128
1 2 0.886
4 2 0.743
8 2 1.202
11 2 0.131
3 3 0.604
6 3 0.361
9 4 0.440
12 4 0.249'''
		for i in Kraw.split("\n"):
			sub = i.split(" ")
			print(sub)
			self.K[int(sub[1])-1][int(sub[0])-1]=float(sub[2])
		print(self.K)

	def getCmd(self,goal):
		return np.dot(self.K,goal-self.state)
	def setState(self,x):
		self.state=np.matrix([[i] for i in x])
