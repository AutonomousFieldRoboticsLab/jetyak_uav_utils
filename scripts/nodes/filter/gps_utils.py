import numpy as np

class GPS_utils:
	'''
		Contains the algorithms to convert a gps signal (longitude, latitude, height)
		to a local cartesian ENU system
		
		Use setENUorigin(lat, lon, height) to set the local ENU coordinate system
		Use geo2ecef(lat, lon, height) to get position in the local ENU system
	'''
	
	def __init__(self):
		# Geodetic System WGS 84 axes
		self.a  = 6378137
		self.b  = 6356752.314245
		self.a2 = self.a * self.a
		self.b2 = self.b * self.b
		self.e2 = 1 - (self.b2 / self.a2)
		
		# Local ENU Origin
		self.latZero = None
		self.lonZero = None
		self.hgtZero = None
		self.xZero = None
		self.yZero = None
		self.zZero = None
		self.R = np.asmatrix(np.eye(3))

	def setENUorigin(self, lat, lon, height):
		# Save origin lat, lon, height
		self.latZero = lat
		self.lonZero = lon
		self.hgtZero = height
		
		# Get origin ECEF X,Y,Z
		origin = self.geo2ecef(self.latZero, self.lonZero, self.hgtZero)		
		self.xZero = origin.item(0)
		self.yZero = origin.item(1)
		self.zZero = origin.item(2)
		self.oZero = np.array([[self.xZero], [self.yZero], [self.zZero]])
		
		# Build rotation matrix
		phi = np.deg2rad(self.latZero)
		lmd = np.deg2rad(self.lonZero)
		
		cPhi = np.cos(phi)
		cLmd = np.cos(lmd)
		sPhi = np.sin(phi)
		sLmd = np.sin(lmd)
		
		self.R[0, 0] = -sLmd
		self.R[0, 1] =  cLmd
		self.R[0, 2] =  0
		self.R[1, 0] = -sPhi*cLmd
		self.R[1, 1] = -sPhi*sLmd
		self.R[1, 2] =  cPhi
		self.R[2, 0] =  cPhi*cLmd
		self.R[2, 1] =  cPhi*sLmd
		self.R[2, 2] =  sPhi
	
	def geo2ecef(self, lat, lon, height):
		phi = np.deg2rad(lat)
		lmd = np.deg2rad(lon)
		
		cPhi = np.cos(phi)
		cLmd = np.cos(lmd)
		sPhi = np.sin(phi)
		sLmd = np.sin(lmd)
		
		N = self.a / (np.sqrt(1 - self.e2*sPhi*sPhi))
		
		x = (N + height)*cPhi*cLmd
		y = (N + height)*cPhi*sLmd
		z = ((self.b2 / self.a2)*N + height)*sPhi
		
		return np.array([[x], [y], [z]])
	
	def ecef2enu(self, x, y, z):
		ecef = np.array([[x], [y], [z]])
		
		return self.R * (ecef - self.oZero)
	
	def geo2enu(self, lat, lon, height):
		ecef = self.geo2ecef(lat, lon, height)
		
		return self.ecef2enu(ecef.item(0), ecef.item(1), ecef.item(2))