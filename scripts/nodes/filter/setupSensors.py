import numpy as np
from sensor import Sensor

def setupSensors(n):
	# Critical chi-squared values for P = 0.001 and different degrees of freedom
	chiSquared_1 = 10.828
	chiSquared_3 = 16.266
	
	# Transition Matrix for Tag measurements
	Htag = np.matrix(np.zeros((4, n)))
	Htag[0:3,   0:3] = np.matrix(-1 * np.eye(3))
	Htag[0:3,   6:9] = np.matrix(np.eye(3))
	Htag[0:3, 11:14] = np.matrix(-1 * np.eye(3))
	Htag[3,      14] = 1

	# Covariance Matrix for Tag measurements
	Rtag = np.asmatrix(1.0e-3 * np.eye(4))

	# Transition Matrix for Drone velocity measurements
	HvelD = np.matrix(np.zeros((3, n)))
	HvelD[0:3, 3:6] = np.matrix(np.eye(3))

	# Covariance Matrix for Drone velocity measurements
	RvelD = np.asmatrix(1.0e-3 * np.eye(3))

	# Transition Matrix for Drone GPS measurements
	HgpsD = np.matrix(np.zeros((3, n)))
	HgpsD[0:3, 0:3] = np.matrix(np.eye(3))

	# Covariance Matrix for Drone GPS measurements
	RgpsD = np.asmatrix(1.0e-1 * np.eye(3))

	# Transition Matrix for Jetyak GPS measurements
	HgpsJ = np.matrix(np.zeros((3, n)))
	HgpsJ[0:3, 6:9] = np.matrix(np.eye(3))

	# Covariance Matrix for Jetyak GPS measurements
	RgpsJ = np.asmatrix(1.0e0 * np.eye(3))

	# Setup sensors
	tagS  = Sensor('tag',  Rtag,  Htag,   39.0, chiSquared_3)
	velDS = Sensor('dvel', RvelD, HvelD, 150.0, chiSquared_1)
	gpsDS = Sensor('dgps', RgpsD, HgpsD, 150.0, chiSquared_1)
	gpsJS = Sensor('jgps', RgpsJ, HgpsJ,  10.0, chiSquared_1)

	return (tagS, velDS, gpsDS, gpsJS)