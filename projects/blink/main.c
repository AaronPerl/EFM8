#include <si_efm8ub1_defs.h>
#include <stdint.h>

int main()
{
	uint32_t i;
	XBR0 = 0x00;
	XBR1 = 0x00;
	XBR2 = 0x40;
	P0MDOUT = 0xFF;
	while (1)
	{
		for (i = 0; i < 10000; i++);
		P0 = 0x00;
		for (i = 0; i < 10000; i++);
		P0 = 0xFF;
	}
}