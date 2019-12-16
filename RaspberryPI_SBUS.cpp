#include <sys/types.h>
#include <iostream>
#include <sys/stat.h>
#include <sys/ipc.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdint.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <asm-generic/termbits.h>
#include <errno.h>
int main(void)
{
	int _device_fd = open("/dev/ttyS0", O_RDWR | O_NONBLOCK | O_CLOEXEC);
	struct termios2 tio { };

	if (0 != ioctl(_device_fd, TCGETS2, &tio)) {
		close(_device_fd);
		_device_fd = -1;
		return -1;
	}
	tio.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL
		| IXON);
	tio.c_iflag |= (INPCK | IGNPAR);
	tio.c_oflag &= ~OPOST;
	tio.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
	tio.c_cflag &= ~(CSIZE | CRTSCTS | PARODD | CBAUD);
	tio.c_cflag |= (CS8 | CSTOPB | CLOCAL | PARENB | BOTHER | CREAD);
	tio.c_ispeed = 100000;
	tio.c_ospeed = 100000;
	tio.c_cc[VMIN] = 25;
	tio.c_cc[VTIME] = 0;

	if (0 != ioctl(_device_fd, TCSETS2, &tio)) {
		close(_device_fd);
		_device_fd = -1;
		return -1;
	}

	int nread;
	uint8_t _sbusData[25]{ 0x0f, 0x01, 0x04, 0x20, 0x00,
					 0xff, 0x07, 0x40, 0x00, 0x02,
					 0x10, 0x80, 0x2c, 0x64, 0x21,
					 0x0b, 0x59, 0x08, 0x40, 0x00,
					 0x02, 0x10, 0x80, 0x00, 0x00 };
	fd_set fds;
	FD_ZERO(&fds);
	FD_SET(_device_fd, &fds);

	while (1) {
		nread = read(_device_fd, &_sbusData, sizeof(_sbusData));
		if (nread == 25) {
			if (0x0f == _sbusData[0] && 0x00 == _sbusData[24]) {
				break;
			}
		}
		usleep(4700);
	}
	for (int i = 0; i < 25; i++)
		std::cout << " " << _sbusData[i];
	std::cout << "\n";
}