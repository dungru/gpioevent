#include <sys/ioctl.h>
#include <fcntl.h>
#include <stdio.h>

#define GPIOEVENT_IOCTL  _IO('T', 0x10)

int main(){
	int fd = open("/dev/gpioevent", O_RDWR);
	int ret;
	while (1) {
		ret = ioctl(fd, GPIOEVENT_IOCTL, 0);    // wait dspevent
		if(ret){
			return ret;
		}
		puts("tone");
	}
}
