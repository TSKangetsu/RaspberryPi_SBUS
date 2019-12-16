#include <unistd.h>
#include <iostream>
#include "RPiSbus.h"
int main(void)
{
	int Channel[16];
	int lose;
	Sbus newSBUS("/dev/ttyS0");
	while (true)
	{
		lose = newSBUS.SbusRead(Channel,4700);
		std::cout << lose << " ";
		for (size_t i = 0; i < 16; i++)
		{
			std::cout << Channel[i] << " ";
		}
		std::cout << "\n";
	}
}