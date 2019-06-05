
#ifndef MANIFOLD_FLASHER_H_
#define MANIFOLD_FLASHER_H_
#include <thread>
#include <unistd.h>
#include "manifoldGPIO.h"
namespace bsc_common {
class Flasher {
public:

	std::thread flasher;
	bool run= false;
	int delay = 100000;

	Flasher() {};
	~Flasher(){};

	void doFlash();

	void startFlash();

	void stopFlash();
};
}
#endif //MANIFOLD_FLASHER_H_
