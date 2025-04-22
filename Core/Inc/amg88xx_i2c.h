#ifndef INC_AMG88XX_I2C_H_
#define INC_AMG88XX_I2C_H_

#include <stdint.h>

/*=========================================================================
 I2C ADDRESS/BITS
 -----------------------------------------------------------------------*/
#define AMG88xx_ADDRESS                (0x68)
/*=========================================================================*/

/*=========================================================================
 REGISTERS
 -----------------------------------------------------------------------*/
enum {
	AMG88xx_PCTL = 0x00,
	AMG88xx_RST = 0x01,
	AMG88xx_FPSC = 0x02,
	AMG88xx_INTC = 0x03,
	AMG88xx_STAT = 0x04,
	AMG88xx_SCLR = 0x05,
	//0x06 reserved
	AMG88xx_AVE = 0x07,
	AMG88xx_INTHL = 0x08,
	AMG88xx_INTHH = 0x09,
	AMG88xx_INTLL = 0x0A,
	AMG88xx_INTLH = 0x0B,
	AMG88xx_IHYSL = 0x0C,
	AMG88xx_IHYSH = 0x0D,
	AMG88xx_TTHL = 0x0E,
	AMG88xx_TTHH = 0x0F,
	AMG88xx_INT_OFFSET = 0x010,
	AMG88xx_PIXEL_OFFSET = 0x80
};

enum amg88xx_power_modes {
	AMG88xx_NORMAL_MODE = 0x00,
	AMG88xx_SLEEP_MODE = 0x01,
	AMG88xx_STAND_BY_60 = 0x20,
	AMG88xx_STAND_BY_10 = 0x21
};

enum sw_resets {
	AMG88xx_FLAG_RESET = 0x30, AMG88xx_INITIAL_RESET = 0x3F
};

enum frame_rates {
	AMG88xx_FPS_10 = 0x00, AMG88xx_FPS_1 = 0x01
};

enum int_enables {
	AMG88xx_INT_DISABLED = 0x00, AMG88xx_INT_ENABLED = 0x01
};

enum int_modes {
	AMG88xx_DIFFERENCE = 0x00, AMG88xx_ABSOLUTE_VALUE = 0x01
};

/*=========================================================================*/

#define AMG88xx_PIXEL_ARRAY_SIZE 64
#define AMG88xx_PIXEL_TEMP_CONVERSION .25f
#define AMG88xx_THERMISTOR_CONVERSION .0625f

/**************************************************************************/
/*! 
 @brief  Class that stores state and functions for interacting with AMG88xx IR sensor chips
 */
/**************************************************************************/

int amg88xx_init(void);
void amg88xx_readPixels(float *buf, uint8_t size/* = AMG88xx_PIXEL_ARRAY_SIZE */);
void amg88xx_readPixelsRaw(int16_t *buf);
float amg88xx_readThermistor(void);
void amg88xx_setMovingAverageMode(int mode);
void amg88xx_enableInterrupt(void);
void amg88xx_disableInterrupt(void);
void amg88xx_setInterruptMode(uint8_t mode);
void amg88xx_getInterrupt(uint8_t *buf, uint8_t /*size = 8 */);
void amg88xx_clearInterrupt(void);

//this will automatically set hysteresis to 95% of the high value
void amg88xx_setInterruptLevels(float high, float low);

//this will manually set hysteresis
void amg88xx_setInterruptLevelsHist(float high, float low, float hysteresis);

void amg88xx_write8(uint8_t reg, uint8_t value);
void amg88xx_write16(uint8_t reg, uint16_t value);
uint8_t amg88xx_read8(uint8_t reg);

void amg88xx_read(uint8_t reg, uint8_t *buf, uint8_t num);
void amg88xx_write(uint8_t reg, uint8_t *buf, uint8_t num);

float amg88xx_signedMag12ToFloat(uint16_t val);

// The power control register
struct amg88xx_pctl {
	// 0x00 = Normal Mode
	// 0x01 = Sleep Mode
	// 0x20 = Stand-by mode (60 sec intermittence)
	// 0x21 = Stand-by mode (10 sec intermittence)         
	uint8_t PCTL :8;
};

//reset register
struct amg88xx_rst {
	//0x30 = flag reset (all clear status reg 0x04, interrupt flag and interrupt table)
	//0x3F = initial reset (brings flag reset and returns to initial setting)
	uint8_t RST :8;
};

//frame rate register
struct amg88xx_fpsc {
	//0 = 10FPS
	//1 = 1FPS
	uint8_t FPS :1;

};

//interrupt control register
struct amg88xx_intc {

	// 0 = INT output reactive (Hi-Z)
	// 1 = INT output active
	uint8_t INTEN :1;

	// 0 = Difference interrupt mode
	// 1 = absolute value interrupt mode
	uint8_t INTMOD :1;

};

//status register
struct amg88xx_stat {
	uint8_t unused :1;
	//interrupt outbreak (val of interrupt table reg)
	uint8_t INTF :1;

	//temperature output overflow (val of temperature reg)
	uint8_t OVF_IRS :1;

	//thermistor temperature output overflow (value of thermistor)
	uint8_t OVF_THS :1;

};

//status clear register
//write to clear overflow flag and interrupt flag
//after writing automatically turns to 0x00
struct amg88xx_sclr {
	uint8_t unused :1;
	//interrupt flag clear
	uint8_t INTCLR :1;
	//temp output overflow flag clear
	uint8_t OVS_CLR :1;
	//thermistor temp output overflow flag clear
	uint8_t OVT_CLR :1;
};

//average register
//for setting moving average output mode
struct amg88xx_ave {
	uint8_t unused :5;
	//1 = twice moving average mode
	uint8_t MAMOD :1;
};

//interrupt level registers
//for setting upper / lower limit hysteresis on interrupt level

//interrupt level upper limit setting. Interrupt output
// and interrupt pixel table are set when value exceeds set value
struct amg88xx_inthl {
	uint8_t INT_LVL_H :8;
};

struct amg88xx_inthh {
	uint8_t INT_LVL_H :4;
};

//interrupt level lower limit. Interrupt output
//and interrupt pixel table are set when value is lower than set value
struct amg88xx_intll {
	uint8_t INT_LVL_L :8;
};

struct amg88xx_intlh {
	uint8_t INT_LVL_L :4;
};

//setting of interrupt hysteresis level when interrupt is generated.
//should not be higher than interrupt level
struct amg88xx_ihysl {
	uint8_t INT_HYS :8;
};

struct amg88xx_ihysh {
	uint8_t INT_HYS :4;
};

//thermistor register
//SIGNED MAGNITUDE FORMAT
struct amg88xx_tthl {
	uint8_t TEMP :8;
};

struct amg88xx_tthh {
	uint8_t TEMP :3;
	uint8_t SIGN :1;
};

#endif // INC_AMG88XX_I2C_H_
