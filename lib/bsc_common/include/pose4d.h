#ifndef JETYAK_UAV_UTILS_POSE4D_H_
#define JETYAK_UAV_UTILS_POSE4D_H_

namespace bsc_common
{
/* A custom encoding for a pose in a 4d space
 * x is x direction
 * y is y direction
 * z is z direction
 * w is yaw
 * t is timestamp in seconds
 */
struct pose4d_t
{
	double t, x, y, z, w;
};
struct vel3d_t
{
	double t, x, y, z;
};
}	// namespace bsc_common
#endif
