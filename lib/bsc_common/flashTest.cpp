// exampleApp.c

#include <unistd.h>
#include "include/flasher.h"

int main(int argc, char *argv[]){

   	bsc_common::Flasher flasher(250000);
		flasher.startFlash();
		
		usleep(10000000);
		flasher.stopFlash();
    return 0;
}


