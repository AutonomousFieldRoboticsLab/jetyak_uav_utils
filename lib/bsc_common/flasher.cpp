#include "include/flasher.h"
namespace bsc_common {
void Flasher::doFlash() {
		manifoldGPIONumber redLED = gpio158 ;     // Ouput

		gpioExport(redLED) ;

		gpioSetDirection(redLED,outputPin) ;


		while(run){

	    gpioSetValue(redLED, on);
	    usleep(delay);         // on for 200ms

	    gpioSetValue(redLED, off);
	    usleep(delay);         // off for 200ms
		}
		gpioUnexport(redLED);     // unexport the LED
	}

	void Flasher::startFlash() {
	  run= true;
		flasher = std::thread(&Flasher::doFlash,this);
	}

	void Flasher::stopFlash() {
		if(run==true) {
			run=false;
			flasher.join();
		}
	}
};
