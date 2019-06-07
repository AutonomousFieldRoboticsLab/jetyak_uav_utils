#include "include/gps_enu.h"

namespace bsc_common
{
/* states
	// Geodetic System WGS 84 Axes
	double a = 6378137;
	double b = 6356752.314245;
	double a2 = a * a;
	double b2 = b * b;
	double e2 = 1 - (b2 / a2);

	// local ENU
	double latZero = null;
	double lonZero = null;
	double altZero = null;
	double xZero = null;
	double yZero = null;
	double zZero = null;
	Eigen::Matrix<double, 3, 3> R;
	*/

GPS_ENU::GPS_ENU()
{
}

void GPS_ENU::setENUOrigin(double lat, double lon, double alt)
{
	//Save origin lat, lon, height
	this->latZero = lat;
	this->lonZero = lon;
	this->altZero = alt;

	//Get origin ECEF X, Y, Z
	Eigen::Matrix<double, 3, 1> origin = this->geo2ecef(this->latZero, this->lonZero, this->altZero);
	this->xZero = origin[0];
	this->yZero = origin[1];
	this->zZero = origin[2];
	this->oZero << this->xZero, this->yZero, this->zZero;

	// Build rotation matrix
	double phi = this->latZero * M_PI / 180.0;
	double lmd = this->lonZero * M_PI / 180.0;

	double cPhi = std::cos(phi);
	double cLmd = std::cos(lmd);
	double sPhi = std::sin(phi);
	double sLmd = std::sin(lmd);

	this->R(0, 0) = -sLmd;
	this->R(0, 1) = cLmd;
	this->R(0, 2) = 0;
	this->R(1, 0) = -sPhi * cLmd;
	this->R(1, 1) = -sPhi * sLmd;
	this->R(1, 2) = cPhi;
	this->R(2, 0) = cPhi * cLmd;
	this->R(2, 1) = cPhi * sLmd;
	this->R(2, 2) = sPhi;
}
Eigen::Matrix<double, 3, 1> GPS_ENU::geo2ecef(double lat, double lon, double alt)
{
	double phi = this->latZero * M_PI / 180.0;
	double lmd = this->lonZero * M_PI / 180.0;

	double cPhi = std::cos(phi);
	double cLmd = std::cos(lmd);
	double sPhi = std::sin(phi);
	double sLmd = std::sin(lmd);

	double N = this->a / std::sqrt(1 - this->e2 * sPhi * sPhi);

	double x = (N + alt) * cPhi * cLmd;
	double y = (N + alt) * cPhi * sLmd;
	double z = ((this->b2 / this->a2) * N + alt) * sPhi;

	Eigen::Matrix<double, 3, 1> ret;
	ret << x, y, z;

	return ret;
}
Eigen::Matrix<double, 3, 1> GPS_ENU::ecef2enu(double x, double y, double z)
{
	Eigen::Matrix<double, 3, 1> ecef(x, y, z);

	return R * (ecef - oZero);
}
Eigen::Matrix<double, 3, 1> GPS_ENU::geo2enu(double lat, double lon, double alt)
{
	Eigen::Matrix<double, 3, 1> ecef = geo2ecef(lat, lon, alt);

	return ecef2enu(ecef(0, 0), ecef(1, 0), ecef(2, 0));
}
} // namespace bsc_common