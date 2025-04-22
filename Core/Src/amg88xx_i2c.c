#include "amg88xx_i2c.h"
#include "stm32f1xx_hal.h"

static struct amg88xx_pctl _pctl;
static struct amg88xx_rst _rst;
static struct amg88xx_fpsc _fpsc;
static struct amg88xx_intc _intc;
static struct amg88xx_stat _stat;
static struct amg88xx_sclr _sclr;
static struct amg88xx_ave _ave;
static struct amg88xx_inthl _inthl;
static struct amg88xx_inthh _inthh;
static struct amg88xx_intll _intll;
static struct amg88xx_intlh _intlh;
static struct amg88xx_ihysl _ihysl;
static struct amg88xx_ihysh _ihysh;
static struct amg88xx_tthl _tthl;
static struct amg88xx_tthh _tthh;

uint8_t amg88xx_getPCTL(void){ return _pctl.PCTL; }
uint8_t amg88xx_getRST(void){ return _rst.RST; }
uint8_t amg88xx_getFPSC(void){ return _fpsc.FPS & 0x01; }
uint8_t amg88xx_getINTC(void){ return (_intc.INTMOD << 1 | _intc.INTEN) & 0x03; }
uint8_t amg88xx_getSTAT(void){ return ( (_stat.OVF_THS << 3) | (_stat.OVF_IRS << 2) | (_stat.INTF << 1) ) & 0x07; }
uint8_t amg88xx_getSCLR(void){ return ((_sclr.OVT_CLR << 3) | (_sclr.OVS_CLR << 2) | (_sclr.INTCLR << 1)) & 0x07; }
uint8_t amg88xx_getAVE(void){ return (_ave.MAMOD << 5); }
uint8_t amg88xx_getINTHL(void){ return _inthl.INT_LVL_H; }
uint8_t amg88xx_getINTHH(void){ return _inthh.INT_LVL_H; }
uint8_t amg88xx_getINTLL(void){ return _intll.INT_LVL_L; }
uint8_t amg88xx_getINTLH(void){ return (_intlh.INT_LVL_L & 0xF); }
uint8_t amg88xx_getIHYSL(void){ return _ihysl.INT_HYS; }
uint8_t amg88xx_getIHYSH(void){ return (_ihysh.INT_HYS & 0xF); }
uint8_t amg88xx_getTTHL(void){ return _tthl.TEMP; }
uint8_t amg88xx_getTTHH(void){ return ( (_tthh.SIGN << 3) | _tthh.TEMP) & 0xF; }
uint8_t amg88xx_min(uint8_t a, uint8_t b){ return a < b ? a : b; }
#define constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))

extern I2C_HandleTypeDef hi2c1;

/**************************************************************************/
/*! 
    @brief  Setups the I2C interface and hardware
    @param  addr Optional I2C address the sensor can be found on. Default is 0x69
    @returns True if device is set up, false on any failure
*/
/**************************************************************************/
int amg88xx_init(void)
{
	//enter normal mode
	_pctl.PCTL = AMG88xx_NORMAL_MODE;
	amg88xx_write8(AMG88xx_PCTL, amg88xx_getPCTL());
	
	//software reset
	_rst.RST = AMG88xx_INITIAL_RESET;
	amg88xx_write8(AMG88xx_RST, amg88xx_getRST());
	
	//disable interrupts by default
	amg88xx_disableInterrupt();
	
	//set to 10 FPS
	_fpsc.FPS = AMG88xx_FPS_10;
	amg88xx_write8(AMG88xx_FPSC, amg88xx_getFPSC());

	HAL_Delay(100);

	return 0;
}

/**************************************************************************/
/*! 
    @brief  Set the moving average mode.
    @param  mode if True is passed, output will be twice the moving average
*/
/**************************************************************************/
void amg88xx_setMovingAverageMode(int mode)
{
	_ave.MAMOD = mode;
	amg88xx_write8(AMG88xx_AVE, amg88xx_getAVE());
}

/**************************************************************************/
/*! 
    @brief  Set the interrupt levels. The hysteresis value defaults to .95 * high
    @param  high the value above which an interrupt will be triggered
    @param  low the value below which an interrupt will be triggered
*/
/**************************************************************************/
void amg88xx_setInterruptLevels(float high, float low)
{
	amg88xx_setInterruptLevelsHist(high, low, high * .95f);
}

/**************************************************************************/
/*! 
    @brief  Set the interrupt levels
    @param  high the value above which an interrupt will be triggered
    @param  low the value below which an interrupt will be triggered
    @param hysteresis the hysteresis value for interrupt detection
*/
/**************************************************************************/
void amg88xx_setInterruptLevelsHist(float high, float low, float hysteresis)
{
	int highConv = high / AMG88xx_PIXEL_TEMP_CONVERSION;
	highConv = constrain(highConv, -4095, 4095);
	_inthl.INT_LVL_H = highConv & 0xFF;
	_inthh.INT_LVL_H = (highConv & 0xF) >> 4;
	amg88xx_write8(AMG88xx_INTHL, amg88xx_getINTHL());
	amg88xx_write8(AMG88xx_INTHH, amg88xx_getINTHH());
	
	int lowConv = low / AMG88xx_PIXEL_TEMP_CONVERSION;
	lowConv = constrain(lowConv, -4095, 4095);
	_intll.INT_LVL_L = lowConv & 0xFF;
	_intlh.INT_LVL_L = (lowConv & 0xF) >> 4;
	amg88xx_write8(AMG88xx_INTLL, amg88xx_getINTLL());
	amg88xx_write8(AMG88xx_INTLH, amg88xx_getINTLH());
	
	int hysConv = hysteresis / AMG88xx_PIXEL_TEMP_CONVERSION;
	hysConv = constrain(hysConv, -4095, 4095);
	_ihysl.INT_HYS = hysConv & 0xFF;
	_ihysh.INT_HYS = (hysConv & 0xF) >> 4;
	amg88xx_write8(AMG88xx_IHYSL, amg88xx_getIHYSL());
	amg88xx_write8(AMG88xx_IHYSH, amg88xx_getIHYSH());
}

/**************************************************************************/
/*! 
    @brief  enable the interrupt pin on the device.
*/
/**************************************************************************/
void amg88xx_enableInterrupt()
{
	_intc.INTEN = 1;
	amg88xx_write8(AMG88xx_INTC, amg88xx_getINTC());
}

/**************************************************************************/
/*! 
    @brief  disable the interrupt pin on the device
*/
/**************************************************************************/
void amg88xx_disableInterrupt()
{
	_intc.INTEN = 0;
	amg88xx_write8(AMG88xx_INTC, amg88xx_getINTC());
}

/**************************************************************************/
/*! 
    @brief  Set the interrupt to either absolute value or difference mode
    @param  mode passing AMG88xx_DIFFERENCE sets the device to difference mode, AMG88xx_ABSOLUTE_VALUE sets to absolute value mode.
*/
/**************************************************************************/
void amg88xx_setInterruptMode(uint8_t mode)
{
	_intc.INTMOD = mode;
	amg88xx_write8(AMG88xx_INTC, amg88xx_getINTC());
}

/**************************************************************************/
/*! 
    @brief  Read the state of the triggered interrupts on the device. The full interrupt register is 8 bytes in length.
    @param  buf the pointer to where the returned data will be stored
    @param  size Optional number of bytes to read. Default is 8 bytes.
    @returns up to 8 bytes of data in buf
*/
/**************************************************************************/
void amg88xx_getInterrupt(uint8_t *buf, uint8_t size)
{
	uint8_t bytesToRead = amg88xx_min(size, (uint8_t)8);
	
	amg88xx_read(AMG88xx_INT_OFFSET, buf, bytesToRead);
}

/**************************************************************************/
/*! 
    @brief  Clear any triggered interrupts
*/
/**************************************************************************/
void amg88xx_clearInterrupt()
{
	_rst.RST = AMG88xx_FLAG_RESET;
	amg88xx_write8(AMG88xx_RST, amg88xx_getRST());
}

/**************************************************************************/
/*! 
    @brief  read the onboard thermistor
    @returns a the floating point temperature in degrees Celsius
*/
/**************************************************************************/
float readThermistor()
{
	uint8_t raw[2];
	amg88xx_read(AMG88xx_TTHL, raw, 2);
	uint16_t recast = ((uint16_t)raw[1] << 8) | ((uint16_t)raw[0]);

	return amg88xx_signedMag12ToFloat(recast) * AMG88xx_THERMISTOR_CONVERSION;
}

/**************************************************************************/
/*! 
    @brief  Read Infrared sensor values
    @param  buf the array to place the pixels in
    @param  size Optionsl number of bytes to read (up to 64). Default is 64 bytes.
    @return up to 64 bytes of pixel data in buf
*/
/**************************************************************************/
void amg88xx_readPixels(float *buf, uint8_t size)
{
	uint16_t recast;
	float converted;
	uint8_t bytesToRead = amg88xx_min((uint8_t)(size << 1), (uint8_t)(AMG88xx_PIXEL_ARRAY_SIZE << 1));
	uint8_t rawArray[bytesToRead];
	amg88xx_read(AMG88xx_PIXEL_OFFSET, rawArray, bytesToRead);
	
	for(int i=0; i<size; i++){
		uint8_t pos = i << 1;
		recast = ((uint16_t)rawArray[pos + 1] << 8) | ((uint16_t)rawArray[pos]);
		
		converted = amg88xx_signedMag12ToFloat(recast) * AMG88xx_PIXEL_TEMP_CONVERSION;
		buf[i] = converted;
	}
}


void amg88xx_readPixelsRaw(int16_t* buf)
{
	amg88xx_read(AMG88xx_PIXEL_OFFSET, (uint8_t*)buf, 128);
}

/**************************************************************************/
/*! 
    @brief  write one byte of data to the specified register
    @param  reg the register to write to
    @param  value the value to write
*/
/**************************************************************************/
void amg88xx_write8(uint8_t reg, uint8_t value)
{
	amg88xx_write(reg, &value, 1);
}

/**************************************************************************/
/*! 
    @brief  read one byte of data from the specified register
    @param  reg the register to read
    @returns one byte of register data
*/
/**************************************************************************/
uint8_t amg88xx_read8(uint8_t reg)
{
	uint8_t ret;
	amg88xx_read(reg, &ret, 1);
	
	return ret;
}


void amg88xx_read(uint8_t reg, uint8_t *buf, uint8_t num)
{
	HAL_StatusTypeDef err;
	
	err = HAL_I2C_Mem_Read(&hi2c1, (AMG88xx_ADDRESS<<1), reg, 1, buf, num, 0xffff);
	if(err != HAL_OK)
		while(1);
}

void amg88xx_write(uint8_t reg, uint8_t *buf, uint8_t num)
{
	HAL_StatusTypeDef err;
	
	err = HAL_I2C_Mem_Write(&hi2c1, (AMG88xx_ADDRESS<<1), reg, 1, buf, num, 0xffff);
	if(err != HAL_OK)
		while(1);
}

/**************************************************************************/
/*! 
    @brief  convert a 12-bit signed magnitude value to a floating point number
    @param  val the 12-bit signed magnitude value to be converted
    @returns the converted floating point value
*/
/**************************************************************************/
float amg88xx_signedMag12ToFloat(uint16_t val)
{
	//take first 11 bits as absolute val
	uint16_t absVal = (val & 0x7FF);
	
	return (val & 0x8000) ? 0 - (float)absVal : (float)absVal ;
}
