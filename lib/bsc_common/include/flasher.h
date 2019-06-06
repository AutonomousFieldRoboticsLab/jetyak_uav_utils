
#ifndef MANIFOLD_FLASHER_H_
#define MANIFOLD_FLASHER_H_
#include <thread>
#include <unistd.h>
#include "manifoldGPIO.h"
namespace bsc_common {
class Flasher {
public:
	manifoldGPIONumber out;
	std::thread flasher;
	bool run= false;
	int delay;

	Flasher(int delay);
	~Flasher();

	void doFlash();

	void startFlash();

	void stopFlash();
};
}
#endif //MANIFOLD_FLASHER_H_
