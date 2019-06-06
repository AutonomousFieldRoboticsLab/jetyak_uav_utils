#include "include/flasher.h"
namespace bsc_common {
Flasher::Flasher(int delay) {
		this->delay = delay;
		out = gpio158 ;     // Ouput

		gpioExport(out) ;

		gpioSetDirection(out,outputPin) ;
			  gpioSetValue(out, off);
}
Flasher::~Flasher() {
		if(run==true) {
			run=false;
			flasher.join();
		}
	  gpioSetValue(out, off);
		
		gpioUnexport(out);     // unexport the LED
}
void Flasher::doFlash() {
		


		while(run){

	    gpioSetValue(out, on);
	    usleep(delay);         // on for 200ms

	    gpioSetValue(out, off);
	    usleep(delay);         // off for 200ms
		}
		
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
