#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <stdint.h>
#include <asm-generic/termbits.h>

#define RCINPUT_MEASURE_INTERVAL_US 4700
#define SBUS_RANGE_MIN 200.0f
#define SBUS_RANGE_MAX 1800.0f
#define SBUS_TARGET_MIN 1000.0f
#define SBUS_TARGET_MAX 2000.0f
#define SBUS_SCALE_FACTOR ((SBUS_TARGET_MAX - SBUS_TARGET_MIN) / (SBUS_RANGE_MAX - SBUS_RANGE_MIN))
#define SBUS_SCALE_OFFSET (int)(SBUS_TARGET_MIN - (SBUS_SCALE_FACTOR * SBUS_RANGE_MIN + 0.5f))

int SBUSInit()
{
	int fd = open("/dev/ttyS0", O_RDWR | O_NONBLOCK | O_CLOEXEC);
	struct termios2 options { };

	if (0 != ioctl(fd, TCGETS2, &options)) {
		close(fd);
		fd = -1;
		return -1;
	}

	options.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL
		| IXON);
	options.c_iflag |= (INPCK | IGNPAR);
	options.c_oflag &= ~OPOST;
	options.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
	options.c_cflag &= ~(CSIZE | CRTSCTS | PARODD | CBAUD);
	options.c_cflag |= (CS8 | CSTOPB | CLOCAL | PARENB | BOTHER | CREAD);
	options.c_ispeed = 100000;
	options.c_ospeed = 100000;
	options.c_cc[VMIN] = 25;
	options.c_cc[VTIME] = 0;

	if (0 != ioctl(fd, TCSETS2, &options)) {
		close(fd);
		fd = -1;
		return -1;
	}
	return fd;
}

void SBUSRead(int fd, int ChannelsData[16], uint8_t SBUS_Raw[25])
{
	uint64_t ts;
	int nread;
	fd_set fds;
	FD_ZERO(&fds);
	FD_SET(fd, &fds);

	while (true) {
		nread = read(fd, &SBUS_Raw, sizeof(SBUS_Raw));
		if (25 == nread) {
			if (0x0f == SBUS_Raw[0] && 0x00 == SBUS_Raw[24]) {
				break;
			}
		}
		usleep(RCINPUT_MEASURE_INTERVAL_US);
	}

	ChannelsData[0] = (uint16_t)(((SBUS_Raw[1] | SBUS_Raw[2] << 8) & 0x07FF)
		* SBUS_SCALE_FACTOR + .5f) + SBUS_SCALE_OFFSET;

	ChannelsData[1] = (uint16_t)(((SBUS_Raw[2] >> 3 | SBUS_Raw[3] << 5)
		& 0x07FF)* SBUS_SCALE_FACTOR + .5f) + SBUS_SCALE_OFFSET;

	ChannelsData[2] = (uint16_t)(((SBUS_Raw[3] >> 6 | SBUS_Raw[4] << 2
		| SBUS_Raw[5] << 10) & 0x07FF)* SBUS_SCALE_FACTOR + .5f)
		+ SBUS_SCALE_OFFSET;

	ChannelsData[3] = (uint16_t)(((SBUS_Raw[5] >> 1 | SBUS_Raw[6] << 7)
		& 0x07FF)* SBUS_SCALE_FACTOR + .5f) + SBUS_SCALE_OFFSET;

	ChannelsData[4] = (uint16_t)(((SBUS_Raw[6] >> 4 | SBUS_Raw[7] << 4)
		& 0x07FF)* SBUS_SCALE_FACTOR + .5f) + SBUS_SCALE_OFFSET;

	ChannelsData[5] = (uint16_t)(((SBUS_Raw[7] >> 7 | SBUS_Raw[8] << 1
		| SBUS_Raw[9] << 9) & 0x07FF)* SBUS_SCALE_FACTOR + .5f)
		+ SBUS_SCALE_OFFSET;

	ChannelsData[6] = (uint16_t)(((SBUS_Raw[9] >> 2 | SBUS_Raw[10] << 6)
		& 0x07FF)* SBUS_SCALE_FACTOR + .5f) + SBUS_SCALE_OFFSET;

	ChannelsData[7] = (uint16_t)(((SBUS_Raw[10] >> 5 | SBUS_Raw[11] << 3)
		& 0x07FF)* SBUS_SCALE_FACTOR + .5f) + SBUS_SCALE_OFFSET;

	ChannelsData[8] = (uint16_t)(((SBUS_Raw[12] | SBUS_Raw[13] << 8)
		& 0x07FF) * SBUS_SCALE_FACTOR + .5f) + SBUS_SCALE_OFFSET;

	ChannelsData[9] = (uint16_t)(((SBUS_Raw[13] >> 3 | SBUS_Raw[14] << 5)
		& 0x07FF)* SBUS_SCALE_FACTOR + .5f) + SBUS_SCALE_OFFSET;

	ChannelsData[10] = (uint16_t)(((SBUS_Raw[14] >> 6 | SBUS_Raw[15] << 2
		| SBUS_Raw[16] << 10) & 0x07FF)* SBUS_SCALE_FACTOR + .5f)
		+ SBUS_SCALE_OFFSET;

	ChannelsData[11] = (uint16_t)(((SBUS_Raw[16] >> 1 | SBUS_Raw[17] << 7)
		& 0x07FF)* SBUS_SCALE_FACTOR + .5f) + SBUS_SCALE_OFFSET;

	ChannelsData[12] = (uint16_t)(((SBUS_Raw[17] >> 4 | SBUS_Raw[18] << 4)
		& 0x07FF)* SBUS_SCALE_FACTOR + .5f) + SBUS_SCALE_OFFSET;

	ChannelsData[13] = (uint16_t)(((SBUS_Raw[18] >> 7 | SBUS_Raw[19] << 1
		| SBUS_Raw[20] << 9) & 0x07FF)* SBUS_SCALE_FACTOR + .5f)
		+ SBUS_SCALE_OFFSET;

	ChannelsData[14] = (uint16_t)(((SBUS_Raw[20] >> 2 | SBUS_Raw[21] << 6)
		& 0x07FF)* SBUS_SCALE_FACTOR + .5f) + SBUS_SCALE_OFFSET;

	ChannelsData[15] = (uint16_t)(((SBUS_Raw[21] >> 5 | SBUS_Raw[22] << 3)
		& 0x07FF)* SBUS_SCALE_FACTOR + .5f) + SBUS_SCALE_OFFSET;
}
