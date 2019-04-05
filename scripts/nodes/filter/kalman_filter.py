import numpy as np

class KalmanFilter:
	"""
		Applies the Kalman Filter Algorithm

		n -> Number of states
		X -> State X = [x y z xdot ydot zdot xddot yddot zddot qX qY qZ qW qXdot qYdot qZdot qWdot]
		F -> State Transition Matrix
		P -> Process Matrix
		Q -> Noise Matrix
		N -> Noise Level (Jerk variance for constant acceleration model)
		R -> Covariance Matrix
		H -> Transition Matrix
	"""

	def __init__(self, n):
		self.n = n
		self.X = None
		self.F = None
		self.P = None
		self.N = None
		self.Q = np.asmatrix(np.zeros((n,n)))

	def initialize(self, X, F, P, N):
		self.X = X
		self.F = F
		self.P = P
		self.N = N

	def updateF(self, dt):
		self.F[0, 3] = dt
		self.F[1, 4] = dt
		self.F[2, 5] = dt
		self.F[3, 6] = dt
		self.F[4, 7] = dt
		self.F[5, 8] = dt
		self.F[0, 6] = 0.5*dt*dt
		self.F[1, 7] = 0.5*dt*dt
		self.F[2, 8] = 0.5*dt*dt
		self.F[9, 13] = dt
		self.F[10, 14] = dt
		self.F[11, 15] = dt


	def updateQ(self, dt):
		dt2 = dt * dt
		dt3 = dt * dt2
		dt4 = dt * dt3
		dt5 = dt * dt4
		dt6 = dt * dt5

		qUD   = dt6 * self.N / 36.0
		qMD   = dt4 * self.N / 4.0
		qLD   = dt2 * self.N
		qTU   = dt5 * self.N / 12.0
		qTL   = dt3 * self.N / 2.0
		qCD   = dt4 * self.N / 6.0

		qx = self.X.item(9)
		qy = self.X.item(10)
		qz = self.X.item(11)
		qw = self.X.item(12)

		G = np.matrix([[ qx,  qy,  qz],
					   [-qw,  qz, -qy],
					   [-qz, -qw,  qx],
					   [ qy, -qx, -qw]])

		Qq = qMD * G * G.T
		Qpos = np.matrix([[qUD, 0, 0, qTU, 0, 0, qCD, 0, 0],
						  [0, qUD, 0, 0, qTU, 0, 0, qCD, 0],
						  [0, 0, qUD, 0, 0, qTU, 0, 0, qCD],
						  [qTU, 0, 0, qMD, 0, 0, qTL, 0, 0],
						  [0, qTU, 0, 0, qMD, 0, 0, qTL, 0],
						  [0, 0, qTU, 0, 0, qMD, 0, 0, qTL],
						  [qCD, 0, 0, qTL, 0, 0, qLD, 0, 0],
						  [0, qCD, 0, 0, qTL, 0, 0, qLD, 0],
						  [0, 0, qCD, 0, 0, qTL, 0, 0, qLD]])
		
		self.Q = np.matrix(np.zeros((self.n, self.n)))
		self.Q[0:9, 0:9] = Qpos
		self.Q[9:13, 9:13] = Qq

	def predict(self):
		self.X = self.F * self.X
		self.P = self.F * self.P * self.F.T + self.Q

		self.checkQuaternion()

	def correct(self, z, H, R):
		hatP = self.P * H.T
		S    = H * hatP + R
		K    = hatP * S.I
		I = np.asmatrix(np.eye(self.n))

		self.X = self.X + K * (z - H * self.X)
		self.P = (I - K * H) * self.P

		self.checkQuaternion()
	
	def checkQuaternion(self):
		qx = self.X.item(9)
		qy = self.X.item(10)
		qz = self.X.item(11)
		qw = self.X.item(12)

		mag = pow(pow(qx,2) + pow(qy,2) + pow(qz,2) + pow(qw,2), 0.5)

		if mag > 0:
			self.X.item(9) / mag
			self.X.item(10) / mag
			self.X.item(11) / mag
			self.X.item(12) / mag

	def getState(self):
		return self.X
