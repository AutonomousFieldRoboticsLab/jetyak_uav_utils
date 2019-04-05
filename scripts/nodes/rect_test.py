import matplotlib.pyplot as plot
from mpl_toolkits.mplot3d import Axes3D
import sys
import math
class Rect():
	def __init__(self,):
		self.corners = {1: (1, 1, 1), 2: (2, 2, 2), 3: (1, 2, 1)}

	def rectangle(self, n):
		assert n>1 , "N must be greater than 1 to form a rectangle"
		norm = lambda L: math.sqrt(sum(i ** 2 for i in L))
		add = lambda v1, v2: [(v1[i] + v2[i]) for i in range(len(v1))]
		scale = lambda v, l: [v[i]*l for i in range(len(v))]
		intermediate=5
		for i in [1, 2, 3]:
			if (self.corners[i] == None):
				return IntResponse(False)
		#n = req.data
		v12 = [(self.corners[2][i] - self.corners[1][i]) for i in [0, 1, 2]]
		v23 = [(self.corners[3][i] - self.corners[2][i]) for i in [0, 1, 2]]
		self.wps = []

		step = scale(v23, 1/float(n-1))
		middle = [add(self.corners[1], scale(v12, i/(intermediate+1.0)))
                    for i in range(1,intermediate+1)]

		left = [add(self.corners[1], scale(step, i)) for i in range(n)]
		right = [add(self.corners[2], scale(step, i)) for i in range(n)]
		mid = [[add(middle[j], scale(step, i)) for i in range(n)]
                    for j in range(intermediate)]
		print mid
		path = []
		for i in range(len(left)):
			if(i % 2 == 0):
				path.append(left.pop(0))
				for j in range(intermediate):
					path.append(mid[j].pop(0))
				path.append(right.pop(0))
			else:
				path.append(right.pop(0))
				for j in range(intermediate)[::-1]:
					path.append(mid[j].pop(0))
				path.append(left.pop(0))

		return path


rg = Rect()
path = rg.rectangle(int(sys.argv[1]))
fig = plot.figure()
ax = fig.add_subplot(111,projection="3d")
ax.scatter([path[i][0] for i in range(len(path))], [path[i][1] for i in range(len(path))], [path[i][2] for i in range(len(path))])
plot.show()
