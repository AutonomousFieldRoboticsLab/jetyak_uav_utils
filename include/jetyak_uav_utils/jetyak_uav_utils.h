
/**
MIT License

Copyright (c) 2018 Brennan Cain and Michail Kalaitzakis (Unmanned Systems and Robotics Lab, University of South Carolina, USA)

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

/**
 * This header provides enumerations for this package.
 * 
 * Author: Brennan Cain
 */
#ifndef JETYAK_UAV_UTILS_H_
#define JETYAK_UAV_UTILS_H_

#include <string>

namespace JETYAK_UAV_UTILS
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
static std::string nameFromMode[] = {"TAKEOFF", "FOLLOW", "LEAVE", "RETURN", "LAND", "RIDE", "HOVER"};
}; // namespace JETYAK_UAV_UTILS

#endif // JETYAK_FLAG_H_