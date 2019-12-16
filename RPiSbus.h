#pragma once
#include <fcntl.h>
#include <string>
#include <unistd.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <stdint.h>
#include <asm-generic/termbits.h>

class Sbus
{
public:
	inline Sbus(char* UartDevice)
	{
		Sbus_fd = open(UartDevice, O_RDWR | O_NONBLOCK | O_CLOEXEC);
		struct termios2 options { };

		if (0 != ioctl(Sbus_fd, TCGETS2, &options)) {
			close(Sbus_fd);
			Sbus_fd = -1;
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

		if (0 != ioctl(Sbus_fd, TCSETS2, &options)) {
			close(Sbus_fd);
			Sbus_fd = -1;
		}
		FD_ZERO(&fd_Maker);
		FD_SET(Sbus_fd, &fd_Maker);
	}

	inline int SbusRead(int* channelsData , int waitTime)
	{
		lose_frameCount = 0;
		while (true) {
			InputBuffer = read(Sbus_fd, &sbusData, sizeof(sbusData));
			if (InputBuffer == 25) {
				if (sbusData[0] == 0x0f && sbusData[24] == 0x00)
					break;
			}
			usleep(waitTime);
			lose_frameCount += 1;
		}

		ChannelsData[0] = (uint16_t)(((sbusData[1] | sbusData[2] << 8) & 0x07FF)
			* sbus_scaler + .5f) + sbus_offset;
		ChannelsData[1] = (uint16_t)(((sbusData[2] >> 3 | sbusData[3] << 5)
			& 0x07FF)* sbus_scaler + .5f) + sbus_offset;
		ChannelsData[2] = (uint16_t)(((sbusData[3] >> 6 | sbusData[4] << 2
			| sbusData[5] << 10) & 0x07FF)* sbus_scaler + .5f)
			+ sbus_offset;
		ChannelsData[3] = (uint16_t)(((sbusData[5] >> 1 | sbusData[6] << 7)
			& 0x07FF)* sbus_scaler + .5f) + sbus_offset;
		ChannelsData[4] = (uint16_t)(((sbusData[6] >> 4 | sbusData[7] << 4)
			& 0x07FF)* sbus_scaler + .5f) + sbus_offset;
		ChannelsData[5] = (uint16_t)(((sbusData[7] >> 7 | sbusData[8] << 1
			| sbusData[9] << 9) & 0x07FF)* sbus_scaler + .5f)
			+ sbus_offset;
		ChannelsData[6] = (uint16_t)(((sbusData[9] >> 2 | sbusData[10] << 6)
			& 0x07FF)* sbus_scaler + .5f) + sbus_offset;
		ChannelsData[7] = (uint16_t)(((sbusData[10] >> 5 | sbusData[11] << 3)
			& 0x07FF)* sbus_scaler + .5f) + sbus_offset;
		ChannelsData[8] = (uint16_t)(((sbusData[12] | sbusData[13] << 8)
			& 0x07FF) * sbus_scaler + .5f) + sbus_offset;
		ChannelsData[9] = (uint16_t)(((sbusData[13] >> 3 | sbusData[14] << 5)
			& 0x07FF)* sbus_scaler + .5f) + sbus_offset;
		ChannelsData[10] = (uint16_t)(((sbusData[14] >> 6 | sbusData[15] << 2
			| sbusData[16] << 10) & 0x07FF)* sbus_scaler + .5f)
			+ sbus_offset;
		ChannelsData[11] = (uint16_t)(((sbusData[16] >> 1 | sbusData[17] << 7)
			& 0x07FF)* sbus_scaler + .5f) + sbus_offset;
		ChannelsData[12] = (uint16_t)(((sbusData[17] >> 4 | sbusData[18] << 4)
			& 0x07FF)* sbus_scaler + .5f) + sbus_offset;
		ChannelsData[13] = (uint16_t)(((sbusData[18] >> 7 | sbusData[19] << 1
			| sbusData[20] << 9) & 0x07FF)* sbus_scaler + .5f)
			+ sbus_offset;
		ChannelsData[14] = (uint16_t)(((sbusData[20] >> 2 | sbusData[21] << 6)
			& 0x07FF)* sbus_scaler + .5f) + sbus_offset;
		ChannelsData[15] = (uint16_t)(((sbusData[21] >> 5 | sbusData[22] << 3)
			& 0x07FF)* sbus_scaler + .5f) + sbus_offset;

		for (size_t i = 0; i < 16; i++)
		{
			channelsData[i] = (int)ChannelsData[i];
		}
		return lose_frameCount;
	}

	inline ~Sbus()
	{
		close(Sbus_fd);
	}
private:
	int Sbus_fd;
	int InputBuffer;
	int lose_frameCount;
	fd_set fd_Maker;
	uint8_t sbusData[25];
	uint16_t ChannelsData[16];
	const double sbus_scaler = (2000.0f - 1000.0f) / (1800.0f - 200.0f);
	const int sbus_offset = 1000.0f - (sbus_scaler * 200.0f + 0.5f);
};
