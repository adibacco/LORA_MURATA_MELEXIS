#include <stdint.h>

#ifdef MLX90640_SAMPLE_DATA

uint16_t ramData_sample[1024];

uint16_t eeData_sample[] = {
		0x00A6,
		0xA99B,
		0x0000,
		0x2061,
		0x0005,
		0x0320,
		0x03E0,
		0x0411,
		0xA232,
		0x0185,
		0x048D,
		0x0000,
		0x1901,
		0x0000,
		0x0000,
		0xBE33,

		0x4210,
		0xFFBB,
		0x0202,
		0xF202,
		0xF2F2,
		0xE2E2,
		0xD1E1,
		0xB1D1,
		0xF10F,
		0xF00F,
		0xE0EF,
		0xE0EF,
		0xE1E1,
		0xF3F2,
		0xF404,
		0xE504,
		0x79A6,
		0x2F44,
		0xFFDD,
		0x2210,
		0x3333,
		0x2233,
		0xEF01,
		0x9ACC,
		0xEEDC,
		0x10FF,
		0x2221,
		0x3333,
		0x2333,
		0x0112,
		0xEEFF,
		0xBBDD,
		0x18EF,
		0x2FF1,
		0x5952,
		0x9D68,
		0x5454,
		0x0994,
		0x6956,
		0x5354,
		0x2363,
		0xE446,
		0xFBB5,
		0x044B,
		0xF020,
		0x9797,
		0x9797,
		0x2889
};

void MLX90640_init_SampleData() {
	ramData_sample[0x056f - 0x0400] = 0x0261;
	ramData_sample[0x0700 - 0x0400] = 0x4BF2;
	ramData_sample[0x0708 - 0x0400] = 0xFFCA;
	ramData_sample[0x0728 - 0x0400] = 0xFFC8;
	ramData_sample[0x070A - 0x0400] = 0x1881;
	ramData_sample[0x0720 - 0x0400] = 0x06AF;
	ramData_sample[0x072A - 0x0400] = 0xCCC5;
	ramData_sample[0x0740 - 0x0400] = 0x1901;

}

#endif


