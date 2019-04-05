#ifndef JETYAK_UAV_H_
#define JETYAK_UAV_H_

#include <string>

namespace JETYAK_UAV
{
// Flag constants. Use these flags internally
enum Flag : uint8_t
{
	YAW_ANGLE = 0b100,
	YAW_RATE = 0b000,
	BODY_FRAME = 0b010,
	WORLD_FRAME = 0b000,
	VELOCITY_CMD = 0b000,
	POSITION_CMD = 0b001
};

// Enumerate the modes
enum Mode : char
{
	TAKEOFF,
	FOLLOW,
	LEAVE,
	RETURN,
	LAND,
	RIDE,
	HOVER
};

// Save the names for human readable display
static std::string nameFromMode[] = { "TAKEOFF", "FOLLOW", "LEAVE", "RETURN", "LAND", "RIDE", "HOVER" };
};	// namespace JETYAK_UAV

#endif	// JETYAK_FLAG_H_