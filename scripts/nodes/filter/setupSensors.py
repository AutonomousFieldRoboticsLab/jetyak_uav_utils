import numpy as np
from sensor import Sensor

def setupSensors(n):
	# Critical chi-squared values for P = 0.001 and different degrees of freedom
	chiSquared_1 = 10.828
	chiSquared_3 = 16.266
	
	# Transition Matrix for Tag measurements
	Htag = np.matrix(np.zeros((4, n)))
	Htag[0:3,   0:3] = np.matrix(-1 * np.eye(3))
	Htag[0:3, 14:17] = np.matrix(np.eye(3))
	Htag[0:3, 20:23] = np.matrix(-1 * np.eye(3))
	Htag[3,      19] =  1
	Htag[3,      23] = -1

	# Covariance Matrix for Tag measurements
	Rtag = np.asmatrix(1.0e-6 * np.eye(4))
	Rtag[3, 3] = 1.0e0

	# Transition Matrix for Attitude measurements
	Hatt = np.matrix(np.zeros((4, n)))
	Hatt[0:4, 6:10] = np.matrix(np.eye(4))

	# Covariance Matrix for Attitude measurements
	Ratt = np.asmatrix(1.0e-9 * np.eye(4))

	# Transition Matrix for IMU measurements
	Himu = np.matrix(np.zeros((4, n)))
	Himu[0:4, 10:14] = np.matrix(np.eye(4))

	# Covariance Matrix for IMU measurements
	Rimu = np.asmatrix(1.0e-6 * np.eye(4))

	# Transition Matrix for Drone velocity measurements
	HvelD = np.matrix(np.zeros((3, n)))
	HvelD[0:3, 3:6] = np.matrix(np.eye(3))

	# Covariance Matrix for Drone velocity measurements
	RvelD = np.asmatrix(1.0e-5 * np.eye(3))

	# Transition Matrix for Drone GPS measurements
	HgpsD = np.matrix(np.zeros((3, n)))
	HgpsD[0:3, 0:3] = np.matrix(np.eye(3))

	# Covariance Matrix for Drone GPS measurements
	RgpsD = np.asmatrix(1.0e-2 * np.eye(3))
	RgpsD[2, 2] = 1.0

	# Transition Matrix for Jetyak GPS measurements
	HgpsJ = np.matrix(np.zeros((3, n)))
	HgpsJ[0:3, 14:17] = np.matrix(np.eye(3))

	# Covariance Matrix for Jetyak GPS measurements
	RgpsJ = np.asmatrix(1.0e-1 * np.eye(3))
	RgpsJ[2, 2] = 1.0

	# Transition Matrix for Jetyak GPS heading
	HhdgJ = np.matrix(np.zeros((1, n)))
	HhdgJ[0, 19] = 1

	# Covariance Matrix for Jetyak GPS heading
	RhdgJ = np.matrix(1.0e-12)

	# Setup sensors
	tagS  = Sensor('tag',  Rtag,  Htag,   39.0, Rtag, chiSquared_3)
	attiS = Sensor('atti', Ratt,  Hatt,  300.0, Ratt, chiSquared_1)
	imuS  = Sensor('imu',  Rimu,  Himu,  300.0, Rimu, chiSquared_1)
	velDS = Sensor('dvel', RvelD, HvelD, 150.0, RvelD, chiSquared_1)
	gpsDS = Sensor('dgps', RgpsD, HgpsD, 150.0, RgpsD, chiSquared_1)
	gpsJS = Sensor('jgps', RgpsJ, HgpsJ,  10.0, RgpsJ, chiSquared_1)
	hdgJS = Sensor('jhdg', RhdgJ, HhdgJ,  10.0, RhdgJ, chiSquared_1)

	return (tagS, attiS, imuS, velDS, gpsDS, gpsJS, hdgJS)