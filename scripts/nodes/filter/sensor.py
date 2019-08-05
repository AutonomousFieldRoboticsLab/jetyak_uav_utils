import numpy as np

class Sensor:
	"""
		Sensor class

		Handles the sensor data and measurements
		Updates R matrix using the residuals
	"""

	def __init__(self, ID, Rnom, H, N, chiCritical):
		self.ID   = ID
		self.R    = Rnom
		self.H    = H
		self.N    = N
		self.Rnom = Rnom
		self.chiC = chiCritical

		self.residuals = None
		self.Z         = None
	
	def setZ(self, measurement):
		self.Z = measurement
	
	def getID(self):
		return self.ID
	
	def getZ(self):
		return self.Z
	
	def getChi(self):
		return self.chiC
	
	def getR(self):
		return self.R
	
	def getH(self):
		return self.H
	
	def updateR(self, r, P):
		if (r is not None) and (P is not None):
			if self.residuals is None:
				self.residuals = np.transpose(r)
			elif self.residuals.shape[0] < self.N:
				self.residuals = np.concatenate((self.residuals, np.transpose(r)), axis = 0)
			else:
				self.residuals = np.concatenate((self.residuals[1:], np.transpose(r)), axis = 0)
			
			if self.residuals.shape[0] == self.N:
				res = np.asmatrix(self.residuals)
				hatRdiag = np.array(np.diagonal(((res.T * res) / self.N) - P))

				for i in range(hatRdiag.shape[0]):
					if hatRdiag[i] < self.Rnom[i,i]:
						hatRdiag[i] = self.Rnom[i,i]
				
				# if self.ID == 'jgps' or self.ID == 'tag':
				# 	print self.ID
				# 	print "Residuals: ", r
				# 	print "P: ", P
				# 	print "New R: ", np.diag(hatRdiag)

				self.R = np.diag(hatRdiag)
