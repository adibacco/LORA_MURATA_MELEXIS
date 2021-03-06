#include <stdint.h>
#include "MLX90640.h"

#ifdef MLX90640_SAMPLE_DATA




uint16_t frameData_sample[] = {
		0xFFB3,
		0xFFAD,
		0xFFB4,
		0xFFAA,
		0xFFB3,
		0xFFAD,
		0xFFB6,
		0xFFA8,
		0xFFB2,
		0xFFA9,
		0xFFB4,
		0xFFA4,
		0xFFB1,
		0xFFA6,
		0xFFB4,
		0xFFA1,
		0xFFB4,
		0xFFA8,
		0xFFB4,
		0xFFA1,
		0xFFB6,
		0xFFA8,
		0xFFB5,
		0xFFA2,
		0xFFBA,
		0xFFA7,
		0xFFB8,
		0xFFA3,
		0xFFB6,
		0xFFAA,
		0xFFBD,
		0xFFA3,
		0xFFAC,
		0xFFA8,
		0xFFA5,
		0xFFA8,
		0xFFAC,
		0xFFA6,
		0xFFA5,
		0xFFA5,
		0xFFAA,
		0xFFA2,
		0xFFA2,
		0xFFA2,
		0xFFAB,
		0xFF9F,
		0xFFA0,
		0xFFA1,
		0xFFAB,
		0xFFA0,
		0xFFA4,
		0xFFA2,
		0xFFAB,
		0xFFA3,
		0xFFA6,
		0xFFA4,
		0xFFB1,
		0xFFA2,
		0xFFA9,
		0xFFA3,
		0xFFB0,
		0xFFA4,
		0xFFAD,
		0xFFA3,
		0xFFB2,
		0xFFAF,
		0xFFB4,
		0xFFA8,
		0xFFB1,
		0xFFAE,
		0xFFB1,
		0xFFA5,
		0xFFB1,
		0xFFAA,
		0xFFB3,
		0xFFA3,
		0xFFB2,
		0xFFA7,
		0xFFB1,
		0xFFA3,
		0xFFB5,
		0xFFA6,
		0xFFB1,
		0xFFA0,
		0xFFB6,
		0xFFA7,
		0xFFB5,
		0xFFA0,
		0xFFB8,
		0xFFA7,
		0xFFB7,
		0xFFA2,
		0xFFB6,
		0xFFA8,
		0xFFB9,
		0xFF9F,
		0xFFAA,
		0xFFA7,
		0xFFA4,
		0xFFA7,
		0xFFA9,
		0xFFA5,
		0xFFA3,
		0xFFA6,
		0xFFA9,
		0xFFA3,
		0xFFA3,
		0xFFA2,
		0xFFAA,
		0xFF9F,
		0xFFA2,
		0xFFA3,
		0xFFAD,
		0xFFA0,
		0xFFA2,
		0xFFA2,
		0xFFAD,
		0xFFA2,
		0xFFA6,
		0xFFA1,
		0xFFB1,
		0xFFA3,
		0xFFA7,
		0xFFA3,
		0xFFAD,
		0xFFA1,
		0xFFA9,
		0xFFA1,
		0xFFB3,
		0xFFAF,
		0xFFB5,
		0xFFAA,
		0xFFB2,
		0xFFAD,
		0xFFB2,
		0xFFA6,
		0xFFB4,
		0xFFAB,
		0xFFB1,
		0xFFA4,
		0xFFB1,
		0xFFA8,
		0xFFB3,
		0xFFA0,
		0xFFB5,
		0xFFA6,
		0xFFB2,
		0xFFA3,
		0xFFB2,
		0xFFA8,
		0xFFB4,
		0xFFA1,
		0xFFB6,
		0xFFA6,
		0xFFB8,
		0xFFA2,
		0xFFB4,
		0xFFA7,
		0xFFB9,
		0xFF9C,
		0xFFAB,
		0xFFA5,
		0xFFA3,
		0xFFA6,
		0xFFAA,
		0xFFA4,
		0xFFA4,
		0xFFA4,
		0xFFB0,
		0xFFA1,
		0xFFA4,
		0xFFA3,
		0xFFA9,
		0xFFA1,
		0xFFA0,
		0xFFA0,
		0xFFAE,
		0xFF9F,
		0xFFA2,
		0xFFA3,
		0xFFAD,
		0xFFA0,
		0xFFA4,
		0xFFA1,
		0xFFAE,
		0xFF9E,
		0xFFA8,
		0xFFA1,
		0xFFAC,
		0xFFA0,
		0xFFA8,
		0xFF9D,
		0xFFB0,
		0xFFAE,
		0xFFB2,
		0xFFA8,
		0xFFB2,
		0xFFAC,
		0xFFB3,
		0xFFA9,
		0xFFB8,
		0xFFB8,
		0xFFB6,
		0xFFAB,
		0xFFB2,
		0xFFAA,
		0xFFB1,
		0xFFA1,
		0xFFB2,
		0xFFA5,
		0xFFB2,
		0xFFA0,
		0xFFB2,
		0xFFA5,
		0xFFB5,
		0xFFA1,
		0xFFB2,
		0xFFA6,
		0xFFB4,
		0xFFA3,
		0xFFB4,
		0xFFA5,
		0xFFB9,
		0xFF9B,
		0xFFA6,
		0xFFA4,
		0xFFA2,
		0xFFA4,
		0xFFA7,
		0xFFA0,
		0xFFA2,
		0xFFA3,
		0xFFBB,
		0xFFA7,
		0xFFAF,
		0xFFAA,
		0xFFAF,
		0xFFA1,
		0xFFA1,
		0xFFA1,
		0xFFA9,
		0xFF9D,
		0xFFA0,
		0xFF9F,
		0xFFA8,
		0xFF9F,
		0xFFA2,
		0xFF9F,
		0xFFAC,
		0xFF9E,
		0xFFA4,
		0xFFA0,
		0xFFAC,
		0xFF9F,
		0xFFA6,
		0xFF9B,
		0xFFAF,
		0xFFAF,
		0xFFB3,
		0xFFA9,
		0xFFAF,
		0xFFAF,
		0xFFB4,
		0xFFB0,
		0xFFBB,
		0xFFC0,
		0xFFC6,
		0xFFB7,
		0xFFC0,
		0xFFB7,
		0xFFB6,
		0xFFA7,
		0xFFB0,
		0xFFA6,
		0xFFB1,
		0xFFA0,
		0xFFB3,
		0xFFA4,
		0xFFB2,
		0xFFA0,
		0xFFB4,
		0xFFA4,
		0xFFB4,
		0xFFA0,
		0xFFB4,
		0xFFA5,
		0xFFB7,
		0xFF9A,
		0xFFA6,
		0xFFA2,
		0xFF9F,
		0xFFA1,
		0xFFA8,
		0xFFA2,
		0xFFAA,
		0xFFA4,
		0xFFBE,
		0xFFB2,
		0xFFB6,
		0xFFB4,
		0xFFBE,
		0xFFAD,
		0xFFAC,
		0xFFA6,
		0xFFAC,
		0xFF9D,
		0xFFA0,
		0xFF9D,
		0xFFA9,
		0xFF9D,
		0xFFA0,
		0xFF9F,
		0xFFAA,
		0xFF9C,
		0xFFA1,
		0xFF9F,
		0xFFAC,
		0xFF9D,
		0xFFA2,
		0xFF9C,
		0xFFB3,
		0xFFAF,
		0xFFB3,
		0xFFAA,
		0xFFB4,
		0xFFB1,
		0xFFB5,
		0xFFBB,
		0xFFC8,
		0xFFC4,
		0xFFC8,
		0xFFBC,
		0xFFC9,
		0xFFBF,
		0xFFC2,
		0xFFB5,
		0xFFBD,
		0xFFB0,
		0xFFB5,
		0xFFA0,
		0xFFB2,
		0xFFA6,
		0xFFB0,
		0xFF9E,
		0xFFB3,
		0xFFA3,
		0xFFB2,
		0xFF9E,
		0xFFB2,
		0xFFA4,
		0xFFB4,
		0xFF9B,
		0xFFA8,
		0xFFA1,
		0xFF9D,
		0xFFA2,
		0xFFAC,
		0xFFA1,
		0xFFB1,
		0xFFAA,
		0xFFC7,
		0xFFB7,
		0xFFB9,
		0xFFB8,
		0xFFC0,
		0xFFB5,
		0xFFB2,
		0xFFB1,
		0xFFBB,
		0xFFA8,
		0xFFA5,
		0xFFA0,
		0xFFA7,
		0xFF9B,
		0xFF9C,
		0xFF9D,
		0xFFA9,
		0xFF9A,
		0xFFA1,
		0xFF9D,
		0xFFA9,
		0xFF9C,
		0xFFA5,
		0xFF9A,
		0xFFAE,
		0xFFAD,
		0xFFAD,
		0xFFA7,
		0xFFB1,
		0xFFB8,
		0xFFBB,
		0xFFC0,
		0xFFCB,
		0xFFCA,
		0xFFCE,
		0xFFC1,
		0xFFCB,
		0xFFBF,
		0xFFC5,
		0xFFB6,
		0xFFC6,
		0xFFB6,
		0xFFBD,
		0xFFA0,
		0xFFB2,
		0xFFA4,
		0xFFB0,
		0xFF9E,
		0xFFB3,
		0xFFA1,
		0xFFB1,
		0xFF9F,
		0xFFB1,
		0xFFA4,
		0xFFB4,
		0xFF9C,
		0xFFA3,
		0xFF9E,
		0xFF9A,
		0xFF9F,
		0xFFAE,
		0xFFA0,
		0xFFB1,
		0xFFB3,
		0xFFC1,
		0xFFBB,
		0xFFB9,
		0xFFBF,
		0xFFBF,
		0xFFB7,
		0xFFB1,
		0xFFB3,
		0xFFB8,
		0xFFB0,
		0xFFA0,
		0xFFA6,
		0xFFA7,
		0xFF9B,
		0xFF9E,
		0xFF99,
		0xFFAB,
		0xFF9A,
		0xFFA0,
		0xFF9D,
		0xFFAA,
		0xFF9D,
		0xFFA2,
		0xFF9A,
		0xFFAC,
		0xFFAF,
		0xFFB0,
		0xFFA9,
		0xFFB1,
		0xFFBD,
		0xFFC3,
		0xFFBA,
		0xFFCA,
		0xFFC2,
		0xFFCD,
		0xFFBC,
		0xFFCA,
		0xFFBE,
		0xFFC8,
		0xFFB7,
		0xFFC6,
		0xFFB1,
		0xFFB9,
		0xFFA0,
		0xFFB2,
		0xFFA5,
		0xFFAF,
		0xFF9D,
		0xFFB3,
		0xFFA4,
		0xFFB0,
		0xFFA0,
		0xFFB0,
		0xFFA3,
		0xFFB4,
		0xFF9B,
		0xFF9E,
		0xFF9C,
		0xFF9C,
		0xFF9F,
		0xFFAE,
		0xFFA4,
		0xFFAE,
		0xFFB3,
		0xFFB8,
		0xFFB5,
		0xFFAF,
		0xFFB9,
		0xFFBA,
		0xFFB3,
		0xFFAF,
		0xFFB2,
		0xFFB4,
		0xFFAF,
		0xFF9D,
		0xFF9D,
		0xFFA5,
		0xFF9B,
		0xFF9D,
		0xFF9A,
		0xFFA9,
		0xFF9A,
		0xFFA1,
		0xFF9C,
		0xFFA8,
		0xFF9B,
		0xFFA3,
		0xFF9A,
		0xFFAE,
		0xFFAF,
		0xFFB0,
		0xFFA8,
		0xFFB6,
		0xFFB5,
		0xFFC0,
		0xFFB5,
		0xFFC3,
		0xFFBC,
		0xFFC2,
		0xFFB6,
		0xFFC4,
		0xFFBD,
		0xFFC2,
		0xFFB0,
		0xFFC4,
		0xFFA9,
		0xFFB2,
		0xFF9E,
		0xFFAE,
		0xFFA3,
		0xFFAE,
		0xFF9E,
		0xFFB0,
		0xFFA3,
		0xFFAF,
		0xFFA1,
		0xFFB2,
		0xFFA6,
		0xFFB3,
		0xFF9E,
		0xFF9E,
		0xFF9A,
		0xFF98,
		0xFF9F,
		0xFFA4,
		0xFFA4,
		0xFFA2,
		0xFFAF,
		0xFFB0,
		0xFFAC,
		0xFFAA,
		0xFFAD,
		0xFFB6,
		0xFFB0,
		0xFFAA,
		0xFFAF,
		0xFFAB,
		0xFFA6,
		0xFF99,
		0xFF9A,
		0xFFA3,
		0xFF98,
		0xFF99,
		0xFF9B,
		0xFFA7,
		0xFF99,
		0xFFA2,
		0xFF9A,
		0xFFAE,
		0xFF9F,
		0xFFA3,
		0xFF9A,
		0xFFA7,
		0xFFAC,
		0xFFAA,
		0xFFA5,
		0xFFAA,
		0xFFAA,
		0xFFB0,
		0xFFA8,
		0xFFB9,
		0xFFB1,
		0xFFBF,
		0xFFB2,
		0xFFBD,
		0xFFB8,
		0xFFC0,
		0xFFA7,
		0xFFBA,
		0xFFA4,
		0xFFAE,
		0xFF9D,
		0xFFAC,
		0xFFA3,
		0xFFAC,
		0xFF9D,
		0xFFB0,
		0xFFA4,
		0xFFB0,
		0xFFAA,
		0xFFBB,
		0xFFB3,
		0xFFBC,
		0xFFA3,
		0xFF99,
		0xFF96,
		0xFF8F,
		0xFF99,
		0xFF97,
		0xFF98,
		0xFF93,
		0xFF9E,
		0xFF9E,
		0xFFA0,
		0xFF9E,
		0xFFA7,
		0xFFAC,
		0xFFA8,
		0xFF9F,
		0xFFA9,
		0xFFA1,
		0xFF9A,
		0xFF94,
		0xFF97,
		0xFFA0,
		0xFF95,
		0xFF98,
		0xFF99,
		0xFFA6,
		0xFF98,
		0xFFA5,
		0xFFA0,
		0xFFB8,
		0xFFA9,
		0xFFAD,
		0xFFA6,
		0xFFA4,
		0xFFAC,
		0xFFA7,
		0xFFA3,
		0xFFA7,
		0xFFA7,
		0xFFA9,
		0xFFA3,
		0xFFAC,
		0xFFA6,
		0xFFB0,
		0xFFA4,
		0xFFB7,
		0xFFAD,
		0xFFB7,
		0xFFA1,
		0xFFAF,
		0xFF9F,
		0xFFA8,
		0xFF9C,
		0xFFAC,
		0xFFA3,
		0xFFAB,
		0xFF9E,
		0xFFAF,
		0xFFAA,
		0xFFB6,
		0xFFAE,
		0xFFBC,
		0xFFBA,
		0xFFC4,
		0xFFA9,
		0xFF95,
		0xFF95,
		0xFF8E,
		0xFF94,
		0xFF96,
		0xFF93,
		0xFF91,
		0xFF96,
		0xFF99,
		0xFF93,
		0xFF94,
		0xFF97,
		0xFF9E,
		0xFF9E,
		0xFF94,
		0xFF9E,
		0xFF9D,
		0xFF93,
		0xFF91,
		0xFF94,
		0xFF9F,
		0xFF96,
		0xFF95,
		0xFF96,
		0xFFA8,
		0xFF97,
		0xFFA6,
		0xFFA5,
		0xFFB6,
		0xFFAA,
		0xFFAB,
		0xFFA6,
		0xFFA4,
		0xFFAA,
		0xFFA4,
		0xFFA2,
		0xFFA6,
		0xFFA7,
		0xFFA6,
		0xFF9E,
		0xFFA5,
		0xFFA4,
		0xFFA4,
		0xFF9F,
		0xFFA7,
		0xFFA1,
		0xFFA7,
		0xFF9B,
		0xFFAB,
		0xFFA0,
		0xFFA9,
		0xFF9D,
		0xFFAA,
		0xFFA1,
		0xFFAB,
		0xFF9F,
		0xFFAE,
		0xFFAE,
		0xFFB7,
		0xFFAD,
		0xFFBB,
		0xFFB5,
		0xFFBB,
		0xFFA3,
		0xFF87,
		0xFF8A,
		0xFF80,
		0xFF8C,
		0xFF8B,
		0xFF8B,
		0xFF83,
		0xFF8B,
		0xFF8B,
		0xFF89,
		0xFF86,
		0xFF8B,
		0xFF8E,
		0xFF89,
		0xFF86,
		0xFF8B,
		0xFF93,
		0xFF8A,
		0xFF87,
		0xFF8D,
		0xFF95,
		0xFF8B,
		0xFF8D,
		0xFF8E,
		0xFFA6,
		0xFF93,
		0xFFA0,
		0xFF9D,
		0xFFA9,
		0xFF9D,
		0xFF9C,
		0xFF99,
		0x4DFA,
		0x1A58,
		0x7FFF,
		0x1A58,
		0x7FFF,
		0x1A57,
		0x7FFF,
		0x1A57,
		0xFFB9,
		0xCE35,
		0x1584,
		0xD64E,
		0xFFF9,
		0x000B,
		0x0000,
		0xFFFE,
		0x1976,
		0x03FD,
		0x0297,
		0x7FFF,
		0x1976,
		0x03FD,
		0x0297,
		0x7FFF,
		0x0001,
		0x0001,
		0x0001,
		0x0001,
		0x0001,
		0x0001,
		0x0001,
		0x0001,
		0x0696,
		0x7FFF,
		0x1A58,
		0x7FFF,
		0x1A58,
		0x7FFF,
		0x1A57,
		0x7FFF,
		0xFFBF,
		0xF57A,
		0xCF18,
		0xD8E0,
		0x000A,
		0xFFFD,
		0xFFFE,
		0x0000,
		0x00F1,
		0x0046,
		0x2AC3,
		0x0035,
		0x00F1,
		0x0046,
		0x2AC3,
		0x0035,
		0x0001,
		0x0001,
		0x0001,
		0x0001,
		0x0001,
		0x0001,
		0x0001,
		0x0001,
		0x1901,
		0x0001
};


#ifdef MLX90640_SAMPLE_DATA_DATASHEET
const uint16_t eeData_sample[] = {
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
#endif
#endif

#ifdef MLX90640_SAMPLE_EEPROM
const uint16_t eeData_sample[] = {
		0x00AE,
		0x499A,
		0x0000,
		0x2061,
		0x0005,
		0x0320,
		0x03E0,
		0x1710,
		0xA224,
		0x0185,
		0x0499,
		0x0000,
		0x1901,
		0x0000,
		0x0000,
		0xB533,
		0x4210,
		0xFFC2,
		0x0202,
		0x0202,
		0xF202,
		0xF1F2,
		0xD1E1,
		0xAFC0,
		0xFF00,
		0xF002,
		0xF103,
		0xE103,
		0xE1F5,
		0xD1E4,
		0xC1D5,
		0x91C2,
		0x8895,
		0x30D9,
		0xEDCB,
		0x110F,
		0x3322,
		0x2233,
		0x0011,
		0xCCEE,
		0xFFED,
		0x1100,
		0x2222,
		0x3333,
		0x2233,
		0x0022,
		0xDEF0,
		0x9ACC,
		0x15CC,
		0x2FA4,
		0x2555,
		0x9C78,
		0x7666,
		0x01C8,
		0x3B38,
		0x3534,
		0x2452,
		0x0463,
		0x13BB,
		0x0623,
		0xEC00,
		0x9797,
		0x9797,
		0x2AFB,
		0x00AE,
		0xFBE0,
		0x1B70,
		0xF3BE,
		0x000E,
		0xF86E,
		0x1B7E,
		0xF3CE,
		0xFFCE,
		0xF41E,
		0x102E,
		0xEC0E,
		0xFFDE,
		0xEC3E,
		0x139E,
		0xEF9E,
		0xFB9E,
		0xF77E,
		0x13E0,
		0xE7EE,
		0xF7AE,
		0xF750,
		0x0C30,
		0xEBEE,
		0xF730,
		0xF010,
		0x0B50,
		0xE430,
		0xF420,
		0xF370,
		0x07C0,
		0xE450,
		0x0470,
		0xFBCE,
		0xFF5C,
		0x0F90,
		0x07D0,
		0xFC3E,
		0xFF6C,
		0x0F90,
		0x03A0,
		0xFC0E,
		0xF40C,
		0x0BF0,
		0x03A0,
		0xF41E,
		0xF78C,
		0x0B70,
		0xFF72,
		0xFF6E,
		0xF7DE,
		0x07C0,
		0xFFA2,
		0x0330,
		0xF42E,
		0x0BC0,
		0xFF22,
		0xFC00,
		0xF75E,
		0x0410,
		0x0022,
		0x0350,
		0xF3A0,
		0x0832,
		0x04DE,
		0xFBF0,
		0x1BCE,
		0xF00E,
		0xFC5E,
		0xFC80,
		0x1BF0,
		0xF02E,
		0x0002,
		0xF81E,
		0x142E,
		0xEC9E,
		0x07DE,
		0xF09E,
		0x17CE,
		0xF3AE,
		0xFFC0,
		0xFBB0,
		0x1080,
		0xEBFE,
		0xFFE0,
		0xFF90,
		0x1460,
		0xE4AE,
		0xFBC0,
		0xF840,
		0x0FE0,
		0xE860,
		0xF8C0,
		0xF400,
		0x0842,
		0xE4B0,
		0x0890,
		0x03BE,
		0xFF9C,
		0x0FD0,
		0x0020,
		0x0450,
		0xFFCC,
		0x0FE0,
		0x07D0,
		0x03FE,
		0xFBEE,
		0x0C60,
		0x0B80,
		0xF86E,
		0xFB8E,
		0x1370,
		0x0782,
		0x038E,
		0xF85E,
		0x0FC2,
		0x07C2,
		0x037E,
		0xF84E,
		0x0880,
		0x0392,
		0x0420,
		0xF7CE,
		0x0C42,
		0xFCB2,
		0xFFE0,
		0xF020,
		0x0490,
		0x107E,
		0x03D0,
		0x1F90,
		0xFBCE,
		0x089E,
		0x0080,
		0x1820,
		0xF40E,
		0x0800,
		0xFC30,
		0x141E,
		0xF06E,
		0x0400,
		0xFFA0,
		0x17CE,
		0xF7B0,
		0x07D0,
		0xFFB0,
		0x1830,
		0xF3FE,
		0x0002,
		0xFFE0,
		0x14D0,
		0xECB0,
		0xFBE2,
		0xFCB0,
		0x13B0,
		0xECA0,
		0xF8DE,
		0xF432,
		0x0832,
		0xE8D0,
		0x1420,
		0xFF8E,
		0xFF6E,
		0x1380,
		0x0840,
		0x005E,
		0xFBEC,
		0x0FB0,
		0x0BB2,
		0xFFFE,
		0xFBDE,
		0x0820,
		0x0BC0,
		0x0360,
		0xFB8C,
		0x0F70,
		0x0794,
		0x036E,
		0xFBFE,
		0x0FA0,
		0x0BC4,
		0x0390,
		0xF89E,
		0x0C72,
		0xFFB2,
		0xFC70,
		0xFB7E,
		0x0470,
		0xFCB0,
		0xFFF0,
		0xF3F0,
		0x04A0,
		0x049E,
		0x03B0,
		0x1F90,
		0xF7D0,
		0x042E,
		0x0070,
		0x1F70,
		0xFBBE,
		0x0F00,
		0x03B0,
		0x142E,
		0xF01E,
		0x07B0,
		0xFFB0,
		0x1B60,
		0xF37E,
		0xFBD0,
		0xFF90,
		0x1410,
		0xF3C0,
		0xFC00,
		0x0370,
		0x1482,
		0xF030,
		0xF800,
		0xFC50,
		0x13C2,
		0xF050,
		0x0070,
		0xF812,
		0x0C02,
		0xEC80,
		0x00D0,
		0xFBFE,
		0xFBCC,
		0x0810,
		0xFC60,
		0xFCB0,
		0xFBCE,
		0x0FE0,
		0x0B40,
		0xFFFE,
		0xF05C,
		0x0840,
		0x07D0,
		0xFFD0,
		0xF79E,
		0x0FB0,
		0xF802,
		0xFFD0,
		0xF44E,
		0x0BF0,
		0xFC32,
		0x07A0,
		0xF4BE,
		0x0C60,
		0xF822,
		0x0080,
		0xF01E,
		0x0892,
		0x00B4,
		0xF850,
		0xF040,
		0x04B2,
		0x085E,
		0x0782,
		0x1F70,
		0xFBEE,
		0x001E,
		0x0420,
		0x1F80,
		0xFBB0,
		0x03B0,
		0x0390,
		0x17F0,
		0xF04E,
		0x0770,
		0xFFE0,
		0x1B40,
		0xF76E,
		0xFFC0,
		0xFFB0,
		0x17E0,
		0xEC1E,
		0x03A0,
		0x03A0,
		0x10C0,
		0xEC60,
		0xFBC2,
		0xFC80,
		0x0C00,
		0xEC60,
		0x0050,
		0xF800,
		0x0802,
		0xEC90,
		0x0080,
		0xF7B0,
		0xF7AE,
		0x0410,
		0xFC32,
		0xFC50,
		0xF7BE,
		0x07F0,
		0xFFD2,
		0xFBC0,
		0xF02E,
		0x0460,
		0x0382,
		0xF410,
		0xF36E,
		0x0BA0,
		0xFBF2,
		0xFBC0,
		0xF01C,
		0x0440,
		0xFFE2,
		0xFBE0,
		0xF0EE,
		0x08A2,
		0xF804,
		0xFCB0,
		0xEC3E,
		0x04A2,
		0x0082,
		0xF830,
		0xE830,
		0x04B2,
		0x13F0,
		0x0380,
		0x1F40,
		0xFBB0,
		0x0F90,
		0x0420,
		0x17A0,
		0xF7AE,
		0x0F40,
		0xFFE2,
		0x13AE,
		0xF03E,
		0x0F12,
		0xFF60,
		0x0F50,
		0xF340,
		0x0362,
		0xFF30,
		0x1760,
		0xEFD0,
		0x0762,
		0x0360,
		0x1072,
		0xEC50,
		0xF7B2,
		0xF852,
		0x07B0,
		0xE480,
		0xF820,
		0xF7C2,
		0x03C2,
		0xE490,
		0x1422,
		0x03AE,
		0x036E,
		0x13C2,
		0x13B2,
		0x0440,
		0xFFCE,
		0x13D2,
		0x1362,
		0x0002,
		0xFBDE,
		0x0C40,
		0x1732,
		0x0390,
		0xFF8E,
		0x1760,
		0x0B82,
		0x0750,
		0x039E,
		0x1000,
		0x0F82,
		0x0B80,
		0xFCAE,
		0x1080,
		0x0BD4,
		0x0470,
		0xFBCE,
		0x0C92,
		0x0832,
		0x07E0,
		0xF7FE,
		0x0CA2,
		0x0010,
		0x0380,
		0x13D0,
		0xF7A0,
		0xFFBE,
		0x0052,
		0x1380,
		0xF770,
		0xFF70,
		0xFFA0,
		0x0FC0,
		0xF3BE,
		0x0340,
		0xFF60,
		0x0FC0,
		0xF370,
		0xFB30,
		0xFB80,
		0x0C10,
		0xE40E,
		0xFBA0,
		0xFBB0,
		0x0C42,
		0xE860,
		0xFB92,
		0xF4A2,
		0x0B82,
		0xE850,
		0xF832,
		0xFBA2,
		0x0002,
		0xE470,
		0x0022,
		0xF7A0,
		0xEFFE,
		0x0BC0,
		0x03D2,
		0xF860,
		0xF79E,
		0x0F92,
		0x0390,
		0xFFB0,
		0xF3FE,
		0x0FC0,
		0x0762,
		0xFF70,
		0xEFFE,
		0x1380,
		0x0362,
		0xFFB0,
		0xF42E,
		0x0810,
		0x07A2,
		0x07C0,
		0xF87E,
		0x0C82,
		0x0B94,
		0x0490,
		0xFB90,
		0x1062,
		0x0842,
		0x07B0,
		0xEC10,
		0x0C82,
		0x0850,
		0x13E2,
		0x2360,
		0x0420,
		0x0460,
		0x10B0,
		0x1FB0,
		0x03E0,
		0x0B80,
		0x0BF0,
		0x1430,
		0xFC00,
		0x0F90,
		0x0BC2,
		0x1BA0,
		0xFFC0,
		0x07C2,
		0x0B82,
		0x1BF0,
		0xF44E,
		0x0BB2,
		0x0FD2,
		0x14C2,
		0xF8A0,
		0x0792,
		0x0852,
		0x13E2,
		0xF850,
		0x00A0,
		0x0032,
		0x0C22,
		0xF0D0,
		0xF452,
		0xEFE0,
		0xEF7E,
		0xFC32,
		0xF072,
		0xF4C0,
		0xEBCE,
		0x03F0,
		0xFBA2,
		0xF400,
		0xE45E,
		0x0410,
		0xFFA2,
		0xF7D0,
		0xEBBE,
		0x0BD0,
		0xFBC2,
		0xFB80,
		0xF00E,
		0x0050,
		0x03D2,
		0x03D0,
		0xF0E0,
		0x0CA0,
		0x0384,
		0x0440,
		0xF3EE,
		0x0C52,
		0x00A2,
		0x0030,
		0xEC20,
		0x04C0,
		0x1022,
		0x0FD2,
		0x1F80,
		0x03F0,
		0x0830,
		0x0C82,
		0x17E0,
		0xFFB0,
		0x0410,
		0x0432,
		0x0870,
		0xF48E,
		0x0BD0,
		0x07B2,
		0x0F90,
		0xFBB0,
		0xFFF0,
		0x07A2,
		0x1410,
		0xF410,
		0x0022,
		0x0BC2,
		0x0CE0,
		0xF850,
		0xFFB2,
		0x0490,
		0x0BC0,
		0xECC0,
		0xFC70,
		0x0012,
		0x0400,
		0xF0B2,
		0x0402,
		0xF7D0,
		0xF37E,
		0x0BF2,
		0x0022,
		0xFC90,
		0xEFFE,
		0x0FC2,
		0xFC12,
		0xF84E,
		0xE87E,
		0x0480,
		0x07E2,
		0xFFB0,
		0xF7AE,
		0x0FC0,
		0x0002,
		0x07A0,
		0xF81E,
		0x1002,
		0x0422,
		0x0FD0,
		0xF8CE,
		0x1842,
		0x07A4,
		0x0880,
		0xFBB0,
		0x0CB0,
		0x0C62,
		0x0BF0,
		0xFBF0,
		0x10A0,
		0xF030,
		0x07D2,
		0x0BE0,
		0xF800,
		0xECA0,
		0x0482,
		0x0830,
		0xFBE0,
		0xF040,
		0xFC80,
		0x0810,
		0xF030,
		0xF410,
		0xF830,
		0x0BA0,
		0xF7A0,
		0xF3D2,
		0xFFF2,
		0x0840,
		0xEFF0,
		0xF400,
		0x03B2,
		0x0872,
		0xF030,
		0xEFB2,
		0x0042,
		0x03B2,
		0xEC40,
		0xFFE0,
		0xFFE2,
		0x0012,
		0xF420,
		0xF422,
		0xF7B0,
		0xE7CE,
		0x0BD2,
		0xF080,
		0x0070,
		0xEC2E,
		0x0FE2,
		0xF850,
		0x0070,
		0xF00E,
		0x0C42,
		0x0020,
		0x0030,
		0xF7AE,
		0x17B2,
		0x03D2,
		0x0400,
		0xF84E,
		0x17F0,
		0x0BE2,
		0x13A0,
		0xFC4E,
		0x1820,
		0x0792,
		0x1020,
		0xFB9E,
		0x1C10,
		0x1BC2,
		0x13C0,
		0xFBE0,
		0x2002,
		0xF040,
		0x13A2,
		0x0F80,
		0xFC30,
		0xF46E,
		0x0CC2,
		0x17B2,
		0x0010,
		0xFC10,
		0x0872,
		0x1000,
		0xF8B0,
		0x07BE,
		0x0BE2,
		0x13B0,
		0xFFE0,
		0xF410,
		0x0450,
		0x0C70,
		0xF420,
		0x03C0,
		0x0F82,
		0x1060,
		0xFFE0,
		0xFB70,
		0x13D2,
		0x0F90,
		0xF820,
		0xFC40,
		0x0FA2,
		0x0BE2,
		0xFC60,
		0xF012,
		0xFB80,
		0xEB5E,
		0x0802,
		0xF420,
		0x0090,
		0xF78E,
		0x13E2,
		0xFC02,
		0x0060,
		0xF40E,
		0x1090,
		0x0F90,
		0x0BD0,
		0xFBAE,
		0x1FD2,
		0x0002,
		0x0820,
		0xF85E,
		0x1800,
		0x0F82,
		0x1B60,
		0xFC3E,
		0x23C2,
		0x0B42,
		0x1BA0,
		0xFF7E,
		0x27E0,
		0x1012,
		0x1B70,
		0xFFC0,
		0x2040,
		0xFC70,
		0x1BA2,
		0x0FA0,
		0x0BA0,
		0x0002,
		0x1432,
		0x0FE0,
		0x0010,
		0xF83E,
		0x13E0,
		0x085E,
		0x07E0,
		0x005E,
		0x0842,
		0x0FEE,
		0x03D0,
		0xFC20,
		0x0FE2,
		0x1400,
		0x0780,
		0x0B90,
		0x1772,
		0x1410,
		0x07B0,
		0xFB10,
		0x17F2,
		0x0B20,
		0x03F0,
		0xFC1E,
		0x17B2,
		0x07CE,
		0x0830,
		0xE050,
		0xEF80,
		0xD38E,
		0x0382,
		0xEBE0,
		0xF810,
		0xDFBE,
		0x07D0,
		0xEC10,
		0xFFC0,
		0xE01E,
		0x0BB0,
		0xF820,
		0xF810,
		0xEBBE,
		0x0BA0,
		0xFBF0,
		0x07A0,
		0xF3EE,
		0x1B50,
		0x0752,
		0x0F30,
		0xF7EE,
		0x1B80,
		0x02F2,
		0x0FD0,
		0xF70E,
		0x13C0,
		0x0BE0,
		0x1390,
		0xF79E,
		0x1C00

};

#endif


void MLX90640_init_SampleData() {


}


