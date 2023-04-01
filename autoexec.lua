----------------------------------------------------------------------
-- ezLCD Pull test application note example
--
-- Created  03/14/2023  -  Jacob Christ
----------------------------------------------------------------------

----------------------------------------------------------------------
-- Lua Shift Function documented here:
-- https://gist.github.com/mebens/938502
----------------------------------------------------------------------
-- function lshift(x, by)
-- 	return x * 2 ^ by
-- end
  
-- function rshift(x, by)
-- 	return math.floor(x / 2 ^ by)
-- end



----------------------------------------------------------------------
-- Lua Enum Function documented here:
-- https://www.lexaloffle.com/bbs/?tid=29891
-- Code here:
-- https://github.com/sulai/Lib-Pico8/blob/master/lang.lua
----------------------------------------------------------------------

function enum(names, offset)
	offset=offset or 1
	local objects = {}
	local size=0
	for idr,name in pairs(names) do
		local id = idr + offset - 1
		local obj = {
			id=id,       -- id
			idr=idr,     -- 1-based relative id, without offset being added
			name=name    -- name of the object
		}
		objects[name] = obj
		objects[id] = obj
		size=size+1
	end
	objects.idstart = offset        -- start of the id range being used
	objects.idend = offset+size-1   -- end of the id range being used
	objects.size=size
	objects.all = function()
		local list = {}
		for _,name in pairs(names) do
			add(list,objects[name])
		end
		local i=0
		return function() i=i+1 if i<=#list then return list[i] end end
	end
	return objects
end



----------------------------------------------------------------------
-- Sparkfun NAU7802 ported to Lua
----------------------------------------------------------------------

-- I2C Connections to Sparkfun QWIIC Scale - ezLCD-5035
-- Pins:
--  SDA - User2 / Pin 2  (SDA)
--  SCL - User2 / Pin 18 (SCL)
--  3V3 - +3.3v (take from RTC Batt Sel)
--  GND - J7 / Pin 1 (GND )

-- /*
--   This is an Arduino library written for the NAU7802 24-bit wheatstone
--   bridge and load cell amplifier.
--   By Nathan Seidle @ SparkFun Electronics, March 3nd, 2019
--   The NAU7802 is an I2C device that converts analog signals to a 24-bit
--   digital signal. This makes it possible to create your own digital scale
--   either by hacking an off-the-shelf bathroom scale or by creating your
--   own scale using a load cell.
--   The NAU7802 is a better version of the popular HX711 load cell amplifier.
--   It uses a true I2C interface so that it can share the bus with other
--   I2C devices while still taking very accurate 24-bit load cell measurements
--   up to 320Hz.
--   https://github.com/sparkfun/SparkFun_NAU7802_Scale_Arduino_Library
--   SparkFun labored with love to create this code. Feel like supporting open
--   source? Buy a board from SparkFun!
--   https://www.sparkfun.com/products/15242
-- */

-- #ifndef SparkFun_Qwiic_Scale_NAU7802_Arduino_Library_h
-- #define SparkFun_Qwiic_Scale_NAU7802_Arduino_Library_h

-- #include "Arduino.h"
-- #include <Wire.h>

-- //Register Map
-- typedef enum
-- {
--   NAU7802_PU_CTRL = 0x00,
--   NAU7802_CTRL1,
--   NAU7802_CTRL2,
--   NAU7802_OCAL1_B2,
--   NAU7802_OCAL1_B1,
--   NAU7802_OCAL1_B0,
--   NAU7802_GCAL1_B3,
--   NAU7802_GCAL1_B2,
--   NAU7802_GCAL1_B1,
--   NAU7802_GCAL1_B0,
--   NAU7802_OCAL2_B2,
--   NAU7802_OCAL2_B1,
--   NAU7802_OCAL2_B0,
--   NAU7802_GCAL2_B3,
--   NAU7802_GCAL2_B2,
--   NAU7802_GCAL2_B1,
--   NAU7802_GCAL2_B0,
--   NAU7802_I2C_CONTROL,
--   NAU7802_ADCO_B2,
--   NAU7802_ADCO_B1,
--   NAU7802_ADCO_B0,
--   NAU7802_ADC = 0x15, //Shared ADC and OTP 32:24
--   NAU7802_OTP_B1,     //OTP 23:16 or 7:0?
--   NAU7802_OTP_B0,     //OTP 15:8
--   NAU7802_PGA = 0x1B,
--   NAU7802_PGA_PWR = 0x1C,
--   NAU7802_DEVICE_REV = 0x1F,
-- } Scale_Registers;

Scale_Registers = enum( 
{
  "NAU7802_PU_CTRL",
  "NAU7802_CTRL1",
  "NAU7802_CTRL2",
  "NAU7802_OCAL1_B2",
  "NAU7802_OCAL1_B1",
  "NAU7802_OCAL1_B0",
  "NAU7802_GCAL1_B3",
  "NAU7802_GCAL1_B2",
  "NAU7802_GCAL1_B1",
  "NAU7802_GCAL1_B0",
  "NAU7802_OCAL2_B2",
  "NAU7802_OCAL2_B1",
  "NAU7802_OCAL2_B0",
  "NAU7802_GCAL2_B3",
  "NAU7802_GCAL2_B2",
  "NAU7802_GCAL2_B1",
  "NAU7802_GCAL2_B0",
  "NAU7802_I2C_CONTROL",
  "NAU7802_ADCO_B2",
  "NAU7802_ADCO_B1",
  "NAU7802_ADCO_B0",
  "NAU7802_ADC", 		-- 0x15 Shared ADC and OTP 32:24
  "NAU7802_OTP_B1",   	-- 0x16 OTP 23:16 or 7:0?
  "NAU7802_OTP_B0",   	-- 0x17 OTP 15:8
  "NAU7802_DUM_0x18",
  "NAU7802_DUM_0x19",
  "NAU7802_DUM_0x1A",
  "NAU7802_PGA",		-- 0x1B,
  "NAU7802_PGA_PWR",	-- 0x1C,
  "NAU7802_DUM_0x1D",
  "NAU7802_DUM_0x1E",
  "NAU7802_DEVICE_REV"	-- 0x1F
}, 0 )

-- //Bits within the PU_CTRL register
-- typedef enum
-- {
--   NAU7802_PU_CTRL_RR = 0,
--   NAU7802_PU_CTRL_PUD,
--   NAU7802_PU_CTRL_PUA,
--   NAU7802_PU_CTRL_PUR,
--   NAU7802_PU_CTRL_CS,
--   NAU7802_PU_CTRL_CR,
--   NAU7802_PU_CTRL_OSCS,
--   NAU7802_PU_CTRL_AVDDS,
-- } PU_CTRL_Bits;
PU_CTRL_Bits = enum( 
{
	"NAU7802_PU_CTRL_RR",
	"NAU7802_PU_CTRL_PUD",
	"NAU7802_PU_CTRL_PUA",
	"NAU7802_PU_CTRL_PUR",
	"NAU7802_PU_CTRL_CS",
	"NAU7802_PU_CTRL_CR",
	"NAU7802_PU_CTRL_OSCS",
	"NAU7802_PU_CTRL_AVDDS"
}, 0)


-- //Bits within the CTRL1 register
-- typedef enum
-- {
--   NAU7802_CTRL1_GAIN = 2,
--   NAU7802_CTRL1_VLDO = 5,
--   NAU7802_CTRL1_DRDY_SEL = 6,
--   NAU7802_CTRL1_CRP = 7,
-- } CTRL1_Bits;

-- //Bits within the CTRL2 register
-- typedef enum
-- {
--   NAU7802_CTRL2_CALMOD = 0,
--   NAU7802_CTRL2_CALS = 2,
--   NAU7802_CTRL2_CAL_ERROR = 3,
--   NAU7802_CTRL2_CRS = 4,
--   NAU7802_CTRL2_CHS = 7,
-- } CTRL2_Bits;
CTRL2_Bits = enum( 
{
	"NAU7802_CTRL2_CALMOD",    -- = 0,
	"NAU7802_CTRL2_DUM_1",     -- = 1,
	"NAU7802_CTRL2_CALS",      -- = 2,
	"NAU7802_CTRL2_CAL_ERROR", -- = 3,
	"NAU7802_CTRL2_CRS",       -- = 4,
	"NAU7802_CTRL2_DUM_5",     -- = 5,
	"NAU7802_CTRL2_DUM_6",     -- = 6,
	"NAU7802_CTRL2_CHS",       -- = 7,
}, 0)

-- //Bits within the PGA register
-- typedef enum
-- {
--   NAU7802_PGA_CHP_DIS = 0,
--   NAU7802_PGA_INV = 3,
--   NAU7802_PGA_BYPASS_EN,
--   NAU7802_PGA_OUT_EN,
--   NAU7802_PGA_LDOMODE,
--   NAU7802_PGA_RD_OTP_SEL,
-- } PGA_Bits;

-- //Bits within the PGA PWR register
-- typedef enum
-- {
--   NAU7802_PGA_PWR_PGA_CURR = 0,
--   NAU7802_PGA_PWR_ADC_CURR = 2,
--   NAU7802_PGA_PWR_MSTR_BIAS_CURR = 4,
--   NAU7802_PGA_PWR_PGA_CAP_EN = 7,
-- } PGA_PWR_Bits;
PGA_PWR_Bits = enum( 
{
	"NAU7802_PGA_PWR_PGA_CURR",       -- = 0,
	"NAU7802_PGA_PWR_DUM_1",          -- = 1,
	"NAU7802_PGA_PWR_ADC_CURR",       -- = 2,
	"NAU7802_PGA_PWR_DUM_3",          -- = 3,
	"NAU7802_PGA_PWR_MSTR_BIAS_CURR", -- = 4,
	"NAU7802_PGA_PWR_DUM_5",          -- = 5,
	"NAU7802_PGA_PWR_DUM_6",          -- = 6,
	"NAU7802_PGA_PWR_PGA_CAP_EN",     -- = 7,
}, 0)

-- //Allowed Low drop out regulator voltages
-- typedef enum
-- {
--   NAU7802_LDO_2V4 = 0b111,
--   NAU7802_LDO_2V7 = 0b110,
--   NAU7802_LDO_3V0 = 0b101,
--   NAU7802_LDO_3V3 = 0b100,
--   NAU7802_LDO_3V6 = 0b011,
--   NAU7802_LDO_3V9 = 0b010,
--   NAU7802_LDO_4V2 = 0b001,
--   NAU7802_LDO_4V5 = 0b000,
-- } NAU7802_LDO_Values;
NAU7802_LDO_Values = enum( 
{
  "NAU7802_LDO_4V5", -- = 0b000,
  "NAU7802_LDO_4V2", -- = 0b001,
  "NAU7802_LDO_3V9", -- = 0b010,
  "NAU7802_LDO_3V6", -- = 0b011,
  "NAU7802_LDO_3V3", -- = 0b100,
  "NAU7802_LDO_3V0", -- = 0b101,
  "NAU7802_LDO_2V7", -- = 0b110,
  "NAU7802_LDO_2V4", -- = 0b111,
}, 0)

-- //Allowed gains
-- typedef enum
-- {
--   NAU7802_GAIN_128 = 0b111,
--   NAU7802_GAIN_64 = 0b110,
--   NAU7802_GAIN_32 = 0b101,
--   NAU7802_GAIN_16 = 0b100,
--   NAU7802_GAIN_8 = 0b011,
--   NAU7802_GAIN_4 = 0b010,
--   NAU7802_GAIN_2 = 0b001,
--   NAU7802_GAIN_1 = 0b000,
-- } NAU7802_Gain_Values;
NAU7802_Gain_Values = enum(
{
	"NAU7802_GAIN_1",   -- = 0b000,
	"NAU7802_GAIN_2",   -- = 0b001,
	"NAU7802_GAIN_4",   -- = 0b010,
	"NAU7802_GAIN_8",   -- = 0b011,
	"NAU7802_GAIN_16",  -- = 0b100,
	"NAU7802_GAIN_32",  -- = 0b101,
	"NAU7802_GAIN_64",  -- = 0b110,
	"NAU7802_GAIN_128", -- = 0b111,
}, 0)


-- //Allowed samples per second
-- typedef enum
-- {
--   NAU7802_SPS_320 = 0b111,
--   NAU7802_SPS_80 = 0b011,
--   NAU7802_SPS_40 = 0b010,
--   NAU7802_SPS_20 = 0b001,
--   NAU7802_SPS_10 = 0b000,
-- } NAU7802_SPS_Values;
NAU7802_SPS_Values = enum(
{
	"NAU7802_SPS_10",  -- = 0b000,
	"NAU7802_SPS_20",  -- = 0b001,
	"NAU7802_SPS_40",  -- = 0b010,
	"NAU7802_SPS_80",  -- = 0b011,
	"NAU7802_SPS_320", -- = 0b111,
}, 0)

-- //Select between channel values
-- typedef enum
-- {
--   NAU7802_CHANNEL_1 = 0,
--   NAU7802_CHANNEL_2 = 1,
-- } NAU7802_Channels;

-- //Calibration state
-- typedef enum
-- {
--   NAU7802_CAL_SUCCESS = 0,
--   NAU7802_CAL_IN_PROGRESS = 1,
--   NAU7802_CAL_FAILURE = 2,
-- } NAU7802_Cal_Status;
NAU7802_Cal_Status = enum(
{
	"NAU7802_CAL_SUCCESS",     -- = 0,
	"NAU7802_CAL_IN_PROGRESS", -- = 1,
	"NAU7802_CAL_FAILURE",     -- = 2,
}, 0)

-- class NAU7802
-- {
-- public:
--   NAU7802();                                               //Default constructor
--   bool begin(TwoWire &wirePort = Wire, bool reset = true); //Check communication and initialize sensor
--   bool isConnected();                                      //Returns true if device acks at the I2C address

--   bool available();                          //Returns true if Cycle Ready bit is set (conversion is complete)
--   int32_t getReading();                      //Returns 24-bit reading. Assumes CR Cycle Ready bit (ADC conversion complete) has been checked by .available()
--   int32_t getAverage(uint8_t samplesToTake); //Return the average of a given number of readings

--   void calculateZeroOffset(uint8_t averageAmount = 8); //Also called taring. Call this with nothing on the scale
--   void setZeroOffset(int32_t newZeroOffset);           //Sets the internal variable. Useful for users who are loading values from NVM.
--   int32_t getZeroOffset();                             //Ask library for this value. Useful for storing value into NVM.

--   void calculateCalibrationFactor(float weightOnScale, uint8_t averageAmount = 8); //Call this with the value of the thing on the scale. Sets the calibration factor based on the weight on scale and zero offset.
--   void setCalibrationFactor(float calFactor);                                      //Pass a known calibration factor into library. Helpful if users is loading settings from NVM.
--   float getCalibrationFactor();                                                    //Ask library for this value. Useful for storing value into NVM.

--   float getWeight(bool allowNegativeWeights = false, uint8_t samplesToTake = 8); //Once you've set zero offset and cal factor, you can ask the library to do the calculations for you.

--   bool setGain(uint8_t gainValue);        //Set the gain. x1, 2, 4, 8, 16, 32, 64, 128 are available
--   bool setLDO(uint8_t ldoValue);          //Set the onboard Low-Drop-Out voltage regulator to a given value. 2.4, 2.7, 3.0, 3.3, 3.6, 3.9, 4.2, 4.5V are avaialable
--   bool setSampleRate(uint8_t rate);       //Set the readings per second. 10, 20, 40, 80, and 320 samples per second is available
--   bool setChannel(uint8_t channelNumber); //Select between 1 and 2

--   bool calibrateAFE();                               //Synchronous calibration of the analog front end of the NAU7802. Returns true if CAL_ERR bit is 0 (no error)
--   void beginCalibrateAFE();                          //Begin asynchronous calibration of the analog front end of the NAU7802. Poll for completion with calAFEStatus() or wait with waitForCalibrateAFE().
--   bool waitForCalibrateAFE(uint32_t timeout_ms = 0); //Wait for asynchronous AFE calibration to complete with optional timeout.
--   NAU7802_Cal_Status calAFEStatus();                 //Check calibration status.

--   bool reset(); //Resets all registers to Power Of Defaults

--   bool powerUp();   //Power up digital and analog sections of scale, ~2mA
--   bool powerDown(); //Puts scale into low-power 200nA mode

--   bool setIntPolarityHigh(); //Set Int pin to be high when data is ready (default)
--   bool setIntPolarityLow();  //Set Int pin to be low when data is ready

--   uint8_t getRevisionCode(); //Get the revision code of this IC. Always 0x0F.

--   bool setBit(uint8_t bitNumber, uint8_t registerAddress);   //Mask & set a given bit within a register
--   bool clearBit(uint8_t bitNumber, uint8_t registerAddress); //Mask & clear a given bit within a register
--   bool getBit(uint8_t bitNumber, uint8_t registerAddress);   //Return a given bit within a register

--   uint8_t getRegister(uint8_t registerAddress);             //Get contents of a register
--   bool setRegister(uint8_t registerAddress, uint8_t value); //Send a given value to be written to given address. Return true if successful

-- private:
--   TwoWire *_i2cPort;                   //This stores the user's requested i2c port
--   const uint8_t _deviceAddress = 0x2A; //Default unshifted 7-bit address of the NAU7802
_deviceAddress = 0x2A -- Default unshifted 7-bit address of the NAU7802

--   //y = mx+b
--   int32_t _zeroOffset;      //This is b
--   float _calibrationFactor; //This is m. User provides this number so that we can output y when requested
-- };
-- #endif

-- /*
--   This is an Arduino library written for the NAU7802 24-bit wheatstone
--   bridge and load cell amplifier.
--   By Nathan Seidle @ SparkFun Electronics, March 3nd, 2019

--   The NAU7802 is an I2C device that converts analog signals to a 24-bit
--   digital signal. This makes it possible to create your own digital scale
--   either by hacking an off-the-shelf bathroom scale or by creating your
--   own scale using a load cell.

--   The NAU7802 is a better version of the popular HX711 load cell amplifier.
--   It uses a true I2C interface so that it can share the bus with other
--   I2C devices while still taking very accurate 24-bit load cell measurements
--   up to 320Hz.

--   https://github.com/sparkfun/SparkFun_Qwiic_Scale_NAU7802_Arduino_Library

--   SparkFun labored with love to create this code. Feel like supporting open
--   source? Buy a board from SparkFun!
--   https://www.sparkfun.com/products/15242
-- */

-- #include "SparkFun_Qwiic_Scale_NAU7802_Arduino_Library.h"

-- //Constructor
-- NAU7802::NAU7802()
-- {
-- }

-- //Sets up the NAU7802 for basic function
-- //If initialize is true (or not specified), default init and calibration is performed
-- //If initialize is false, then it's up to the caller to initalize and calibrate
-- //Returns true upon completion
-- bool NAU7802::begin(TwoWire &wirePort, bool initialize)
-- {
--   //Get user's options
--   _i2cPort = &wirePort;

--   //Check if the device ack's over I2C
--   if (isConnected() == false)
--   {
--     //There are rare times when the sensor is occupied and doesn't ack. A 2nd try resolves this.
--     if (isConnected() == false)
--       return (false);
--   }

--   bool result = true; //Accumulate a result as we do the setup

--   if (initialize)
--   {
--     result &= reset(); //Reset all registers

--     result &= powerUp(); //Power on analog and digital sections of the scale

--     result &= setLDO(NAU7802_LDO_3V3); //Set LDO to 3.3V

--     result &= setGain(NAU7802_GAIN_128); //Set gain to 128

--     result &= setSampleRate(NAU7802_SPS_80); //Set samples per second to 10

--     result &= setRegister(NAU7802_ADC, 0x30); //Turn off CLK_CHP. From 9.1 power on sequencing.

--     result &= setBit(NAU7802_PGA_PWR_PGA_CAP_EN, NAU7802_PGA_PWR); //Enable 330pF decoupling cap on chan 2. From 9.14 application circuit note.

--     result &= calibrateAFE(); //Re-cal analog front end when we change gain, sample rate, or channel
--   }

--   return (result);
-- }
function NAU7802_begin(initialize) -- return boolean
	local result, str

	-- Check if the device ack's over I2C
	if NAU7802_isConnected() == false then
		-- There are rare times when the sensor is occupied and doesn't ack. A 2nd try resolves this.
		if NAU7802_isConnected() == false then
			return false
		end
	end

	result = true -- Accumulate a result as we do the setup

	if initialize == true then
		str = "NAU7802_reset"
		ez.SerialTx(str .. "\r\n", 80, debug_port)
		result = result and NAU7802_reset()
		str = "NAU7802_powerUp"
		ez.SerialTx(str .. "\r\n", 80, debug_port)
		result = result and NAU7802_powerUp() -- Power on analog and digital sections of the scale
		str = "NAU7802_setLDO"
		ez.SerialTx(str .. "\r\n", 80, debug_port)
	    result = result and NAU7802_setLDO(NAU7802_LDO_Values.NAU7802_LDO_3V3.id) -- Set LDO to 3.3V
		str = "NAU7802_setGain"
		ez.SerialTx(str .. "\r\n", 80, debug_port)
		result = result and NAU7802_setGain(NAU7802_Gain_Values.NAU7802_GAIN_128.id) -- Set gain to 128
		str = "NAU7802_setSampleRate"
		ez.SerialTx(str .. "\r\n", 80, debug_port)
		result = result and NAU7802_setSampleRate(NAU7802_SPS_Values.NAU7802_SPS_80.id) -- Set samples per second to 10
		str = "NAU7802_setRegister"
		ez.SerialTx(str .. "\r\n", 80, debug_port)
		result = result and NAU7802_setRegister(Scale_Registers.NAU7802_ADC.id, 0x30) -- Turn off CLK_CHP. From 9.1 power on sequencing.
		str = "NAU7802_setBit"
		ez.SerialTx(str .. "\r\n", 80, debug_port)
		result = result and NAU7802_setBit(PGA_PWR_Bits.NAU7802_PGA_PWR_PGA_CAP_EN.id, Scale_Registers.NAU7802_PGA_PWR.id) -- Enable 330pF decoupling cap on chan 2. From 9.14 application circuit note.
		str = "NAU7802_calibrateAFE"
		ez.SerialTx(str .. "\r\n", 80, debug_port)
		result = result and NAU7802_calibrateAFE() -- Re-cal analog front end when we change gain, sample rate, or channel
		str = "done."
		ez.SerialTx(str .. "\r\n", 80, debug_port)
	end

   return result
end

-- //Returns true if device is present
-- //Tests for device ack to I2C address
-- bool NAU7802::isConnected()
-- {
--   _i2cPort->beginTransmission(_deviceAddress);
--   if (_i2cPort->endTransmission() != 0)
--     return (false); //Sensor did not ACK
--   return (true);    //All good
-- }
function NAU7802_isConnected()
	local result
	result = ez.I2Cread(_deviceAddress,0)
	if result == nil then
		-- result = "nil"
		-- ez.SetXY(50, 150)
		-- print("isConnected:", result)
		return false
	else
		-- ez.SetXY(50, 150)
		-- print("isConnected:" .. string.byte(result, 1))
		-- print("string.len:" .. string.len(result))
		return true
	end
end


-- //Returns true if Cycle Ready bit is set (conversion is complete)
-- bool NAU7802::available()
-- {
--   return (getBit(NAU7802_PU_CTRL_CR, NAU7802_PU_CTRL));
-- }
function NAU7802_available() -- bool NAU7802::available()
	if NAU7802_getBit(PU_CTRL_Bits.NAU7802_PU_CTRL_CR.id, Scale_Registers.NAU7802_PU_CTRL.id) == 0 then
		return false
	end
	return true
end
	

-- //Calibrate analog front end of system. Returns true if CAL_ERR bit is 0 (no error)
-- //Takes approximately 344ms to calibrate; wait up to 1000ms.
-- //It is recommended that the AFE be re-calibrated any time the gain, SPS, or channel number is changed.
-- bool NAU7802::calibrateAFE()
-- {
--   beginCalibrateAFE();
--   return waitForCalibrateAFE(1000);
-- }
function NAU7802_calibrateAFE() -- bool NAU7802::calibrateAFE()
	NAU7802_beginCalibrateAFE()
	return NAU7802_waitForCalibrateAFE(1000)
end

-- //Begin asynchronous calibration of the analog front end.
-- // Poll for completion with calAFEStatus() or wait with waitForCalibrateAFE()
-- void NAU7802::beginCalibrateAFE()
-- {
--   setBit(NAU7802_CTRL2_CALS, NAU7802_CTRL2);
-- }
function NAU7802_beginCalibrateAFE() -- void NAU7802::beginCalibrateAFE()
	NAU7802_setBit(CTRL2_Bits.NAU7802_CTRL2_CALS.id, Scale_Registers.NAU7802_CTRL2.id);
end

-- //Check calibration status.
-- NAU7802_Cal_Status NAU7802::calAFEStatus()
-- {
--   if (getBit(NAU7802_CTRL2_CALS, NAU7802_CTRL2))
--   {
--     return NAU7802_CAL_IN_PROGRESS;
--   }

--   if (getBit(NAU7802_CTRL2_CAL_ERROR, NAU7802_CTRL2))
--   {
--     return NAU7802_CAL_FAILURE;
--   }

--   // Calibration passed
--   return NAU7802_CAL_SUCCESS;
-- }
function NAU7802_calAFEStatus() -- NAU7802_Cal_Status NAU7802::calAFEStatus()
	local str
	str = "NAU7802_calAFEStatus 1"
	ez.SerialTx(str .. "\r\n", 80, debug_port)
	if (NAU7802_getBit(CTRL2_Bits.NAU7802_CTRL2_CALS.id, Scale_Registers.NAU7802_CTRL2.id)) then
		return NAU7802_Cal_Status.NAU7802_CAL_IN_PROGRESS.id
	end
	str = "NAU7802_calAFEStatus 2"
	ez.SerialTx(str .. "\r\n", 80, debug_port)

	if (NAU7802_getBit(CTRL2_Bits.NAU7802_CTRL2_CAL_ERROR.id, Scale_Registers.NAU7802_CTRL2.id)) then
		return NAU7802_Cal_Status.NAU7802_CAL_FAILURE.id
	end
	str = "NAU7802_calAFEStatus 3"
	ez.SerialTx(str .. "\r\n", 80, debug_port)

	-- Calibration passed
	return NAU7802_Cal_Status.NAU7802_CAL_SUCCESS.id
end

-- //Wait for asynchronous AFE calibration to complete with optional timeout.
-- //If timeout is not specified (or set to 0), then wait indefinitely.
-- //Returns true if calibration completes succsfully, otherwise returns false.
-- bool NAU7802::waitForCalibrateAFE(uint32_t timeout_ms)
-- {
--   uint32_t begin = millis();
--   NAU7802_Cal_Status cal_ready;

--   while ((cal_ready = calAFEStatus()) == NAU7802_CAL_IN_PROGRESS)
--   {
--     if ((timeout_ms > 0) && ((millis() - begin) > timeout_ms))
--     {
--       break;
--     }
--     delay(1);
--   }

--   if (cal_ready == NAU7802_CAL_SUCCESS)
--   {
--     return (true);
--   }
--   return (false);
-- }
function NAU7802_waitForCalibrateAFE(timeout_ms) -- bool NAU7802::waitForCalibrateAFE(uint32_t timeout_ms)
	local str
	local begin = ez.Get_ms() --   uint32_t begin = millis();
	local cal_ready --   NAU7802_Cal_Status cal_ready;

	cal_ready = NAU7802_calAFEStatus()
	while (cal_ready == NAU7802_Cal_Status.NAU7802_CAL_IN_PROGRESS.id) do
		str = "cal_ready = " .. tostring(cal_ready)
		ez.SerialTx(str .. "\r\n", 80, debug_port)

		if ((timeout_ms > 0) and ((ez.Get_ms() - begin) > timeout_ms)) then
			str = "break"
			ez.SerialTx(str .. "\r\n", 80, debug_port)
				break
		end
		ez.Wait_ms(1)
		cal_ready = NAU7802_calAFEStatus()
	end

	if (cal_ready == NAU7802_Cal_Status.NAU7802_CAL_SUCCESS.id) then
		return true
	end
	return false
end

-- //Set the readings per second
-- //10, 20, 40, 80, and 320 samples per second is available
-- bool NAU7802::setSampleRate(uint8_t rate)
-- {
--   if (rate > 0b111)
--     rate = 0b111; //Error check

--   uint8_t value = getRegister(NAU7802_CTRL2);
--   value &= 0b10001111; //Clear CRS bits
--   value |= rate << 4;  //Mask in new CRS bits

--   return (setRegister(NAU7802_CTRL2, value));
-- }
function NAU7802_setSampleRate(rate) -- bool NAU7802::setSampleRate(uint8_t rate)
	local value
	if (rate > 0x7) then
		rate = 0x7 -- Error check
	end

	value = NAU7802_getRegister(Scale_Registers.NAU7802_CTRL2.id);
	value = value & 0x8F -- 0b10001111; //Clear CRS bits
	value = value | ((rate << 4) & 0xFF) -- Mask in new CRS bits
	return NAU7802_setRegister(Scale_Registers.NAU7802_CTRL2.id, value)
end

-- //Select between 1 and 2
-- bool NAU7802::setChannel(uint8_t channelNumber)
-- {
--   if (channelNumber == NAU7802_CHANNEL_1)
--     return (clearBit(NAU7802_CTRL2_CHS, NAU7802_CTRL2)); //Channel 1 (default)
--   else
--     return (setBit(NAU7802_CTRL2_CHS, NAU7802_CTRL2)); //Channel 2
-- }

-- //Power up digital and analog sections of scale
-- bool NAU7802::powerUp()
-- {
--   setBit(NAU7802_PU_CTRL_PUD, NAU7802_PU_CTRL);
--   setBit(NAU7802_PU_CTRL_PUA, NAU7802_PU_CTRL);

--   //Wait for Power Up bit to be set - takes approximately 200us
--   uint8_t counter = 0;
--   while (1)
--   {
--     if (getBit(NAU7802_PU_CTRL_PUR, NAU7802_PU_CTRL) == true)
--       break; //Good to go
--     delay(1);
--     if (counter++ > 100)
--       return (false); //Error
--   }
--   return (true);
-- }
function NAU7802_powerUp() -- bool NAU7802::powerUp()
	local counter
	NAU7802_setBit(PU_CTRL_Bits.NAU7802_PU_CTRL_PUD.id, Scale_Registers.NAU7802_PU_CTRL.id)
	NAU7802_setBit(PU_CTRL_Bits.NAU7802_PU_CTRL_PUA.id, Scale_Registers.NAU7802_PU_CTRL.id)

	-- Wait for Power Up bit to be set - takes approximately 200us
	counter = 0 --	uint8_t counter = 0;
	while 1==1 do
		if NAU7802_getBit(PU_CTRL_Bits.NAU7802_PU_CTRL_PUR.id, Scale_Registers.NAU7802_PU_CTRL.id) == true then
			break	-- Good to go
		end
		ez.Wait_ms(1)
		if (counter > 100) then
			return false -- Error
		end
		counter = counter + 1
	end
	return true
end



-- //Puts scale into low-power mode
-- bool NAU7802::powerDown()
-- {
--   clearBit(NAU7802_PU_CTRL_PUD, NAU7802_PU_CTRL);
--   return (clearBit(NAU7802_PU_CTRL_PUA, NAU7802_PU_CTRL));
-- }

-- //Resets all registers to Power Of Defaults
-- bool NAU7802::reset()
-- {
--   setBit(NAU7802_PU_CTRL_RR, NAU7802_PU_CTRL); //Set RR
--   delay(1);
--   return (clearBit(NAU7802_PU_CTRL_RR, NAU7802_PU_CTRL)); //Clear RR to leave reset state
-- }
function NAU7802_reset()
	NAU7802_setBit(PU_CTRL_Bits.NAU7802_PU_CTRL_RR.id, Scale_Registers.NAU7802_PU_CTRL.id) -- Set RR
	ez.Wait_ms(1)
	return (NAU7802_clearBit(PU_CTRL_Bits.NAU7802_PU_CTRL_RR.id, Scale_Registers.NAU7802_PU_CTRL.id)) -- Clear RR to leave reset state
end

-- //Set the onboard Low-Drop-Out voltage regulator to a given value
-- //2.4, 2.7, 3.0, 3.3, 3.6, 3.9, 4.2, 4.5V are available
-- bool NAU7802::setLDO(uint8_t ldoValue)
-- {
--   if (ldoValue > 0b111)
--     ldoValue = 0b111; //Error check

--   //Set the value of the LDO
--   uint8_t value = getRegister(NAU7802_CTRL1);
--   value &= 0b11000111;    //Clear LDO bits
--   value |= ldoValue << 3; //Mask in new LDO bits
--   setRegister(NAU7802_CTRL1, value);

--   return (setBit(NAU7802_PU_CTRL_AVDDS, NAU7802_PU_CTRL)); //Enable the internal LDO
-- }
function NAU7802_setLDO(ldoValue) -- bool NAU7802::setLDO(uint8_t ldoValue)
	local value
	if (ldoValue > 0x7) then
		ldoValue = 0x7 -- Error check
	end

	-- Set the value of the LDO
	value = NAU7802_getRegister(Scale_Registers.NAU7802_CTRL1.id);
	value = value & 0xC7 -- 0b11000111 Clear LDO bits
	value = value | ((ldoValue << 3) & 0xFF) -- Mask in new LDO bits
	NAU7802_setRegister(Scale_Registers.NAU7802_CTRL1.id, value)

	return (NAU7802_setBit(PU_CTRL_Bits.NAU7802_PU_CTRL_AVDDS.id, Scale_Registers.NAU7802_PU_CTRL.id)) -- Enable the internal LDO
end

-- //Set the gain
-- //x1, 2, 4, 8, 16, 32, 64, 128 are avaialable
-- bool NAU7802::setGain(uint8_t gainValue)
-- {
--   if (gainValue > 0b111)
--     gainValue = 0b111; //Error check

--   uint8_t value = getRegister(NAU7802_CTRL1);
--   value &= 0b11111000; //Clear gain bits
--   value |= gainValue;  //Mask in new bits

--   return (setRegister(NAU7802_CTRL1, value));
-- }
function NAU7802_setGain(gainValue) -- bool NAU7802::setGain(uint8_t gainValue)
	local value
	if (gainValue > 0x7) then
		gainValue = 0x7 -- Error check
	end

	value = NAU7802_getRegister(Scale_Registers.NAU7802_CTRL1.id);
	value = value & 0xF8 -- 0b11111000; //Clear gain bits
	value = value | ((gainValue) & 0xFF) -- Mask in new LDO bits
	return NAU7802_setRegister(Scale_Registers.NAU7802_CTRL1.id, value)
end

-- //Get the revision code of this IC
-- uint8_t NAU7802::getRevisionCode()
-- {
--   uint8_t revisionCode = getRegister(NAU7802_DEVICE_REV);
--   return (revisionCode & 0x0F);
-- }

-- //Returns 24-bit reading
-- //Assumes CR Cycle Ready bit (ADC conversion complete) has been checked to be 1
-- int32_t NAU7802::getReading()
-- {
--   _i2cPort->beginTransmission(_deviceAddress);
--   _i2cPort->write(NAU7802_ADCO_B2);
--   if (_i2cPort->endTransmission() != 0)
--     return (false); //Sensor did not ACK

--   _i2cPort->requestFrom((uint8_t)_deviceAddress, (uint8_t)3);

--   if (_i2cPort->available())
--   {
--     uint32_t valueRaw = (uint32_t)_i2cPort->read() << 16; //MSB
--     valueRaw |= (uint32_t)_i2cPort->read() << 8;          //MidSB
--     valueRaw |= (uint32_t)_i2cPort->read();               //LSB

--     // the raw value coming from the ADC is a 24-bit number, so the sign bit now
--     // resides on bit 23 (0 is LSB) of the uint32_t container. By shifting the
--     // value to the left, I move the sign bit to the MSB of the uint32_t container.
--     // By casting to a signed int32_t container I now have properly recovered
--     // the sign of the original value
--     int32_t valueShifted = (int32_t)(valueRaw << 8);

--     // shift the number back right to recover its intended magnitude
--     int32_t value = (valueShifted >> 8);

--     return (value);
--   }

--   return (0); //Error
-- }
function NAU7802_getReading() -- int32_t NAU7802::getReading()
	local str
	local result
	local result1, result2, result3

	-- str = "NAU7802_getReading "
	-- ez.SerialTx(str, 80, debug_port)

	-- result = ez.I2Cread(_deviceAddress,Scale_Registers.NAU7802_ADCO_B2.id,3)

	-- result[1] = NAU7802_getRegister(Scale_Registers.NAU7802_ADCO_B2.id)
	-- result[2] = NAU7802_getRegister(Scale_Registers.NAU7802_ADCO_B1.id)
	-- result[3] = NAU7802_getRegister(Scale_Registers.NAU7802_ADCO_B0.id)

	result1 = NAU7802_getRegister(Scale_Registers.NAU7802_ADCO_B2.id)
	result2 = NAU7802_getRegister(Scale_Registers.NAU7802_ADCO_B1.id)
	result3 = NAU7802_getRegister(Scale_Registers.NAU7802_ADCO_B0.id)

	-- result = {result1, result2, result3}

	-- str = "#result = " .. tostring(#result)
	-- ez.SerialTx(str .. "\r\n", 80, debug_port)

	-- for i = 1,3,1 do
	-- 	str = "reslut["..tostring(i).."] = "
	-- 	ez.SerialTx(str, 80, debug_port)
	-- 	str = string.format("%02X", result[1])
	-- 	-- str = string.byte(result[i]) -- .format("%02X", result[1])
	-- 	-- str = type(result[i]) -- .format("%02X", result[1])
	-- 	ez.SerialTx(str .. "\r\n", 80, debug_port)
			
	-- end

	if result3 ~= nil then
		local rawValue = 0
		local bit
		local mask
		mask = ((1 << 23) & 0xffffffff)

		rawValue = rawValue | result1 << 16		-- MSB
		rawValue = rawValue | result2 << 8		-- MidSB
		rawValue = rawValue | result3			-- LSB

		-- str = "rawValue = " .. string.format("%06X", rawValue) .. " "
		-- ez.SerialTx(str, 80, debug_port)

		-- str = "mask = " .. string.format("%06X", mask) .. " "
		-- ez.SerialTx(str, 80, debug_port)

		bit = (rawValue & mask) ~= 0 and true or false

		-- str = "bit = " .. tostring(bit) .. " "
		-- ez.SerialTx(str, 80, debug_port)

		local valueShifted = rawValue << (8)

		-- str = "valueShifted = " .. string.format("%06X", valueShifted) .. " "
		-- ez.SerialTx(str, 80, debug_port)


		if 	bit then
			valueShifted = ~valueShifted
			-- str = "~" .. string.format("%06X", valueShifted) .. " "
			-- ez.SerialTx(str, 80, debug_port)
			valueShifted = valueShifted + 0x100
			-- str = "+" .. string.format("%06X", valueShifted) .. " "
			-- ez.SerialTx(str, 80, debug_port)
		end

		-- shift the number back right to recover its intended magnitude
		valueShifted = valueShifted >> (8)

		if 	bit then
			valueShifted = valueShifted * -1
			-- str = string.format("%06X", valueShifted) .. " "
			-- ez.SerialTx(str, 80, debug_port)
		end

		-- str = "value = " .. string.format("%08X", valueShifted)
		-- ez.SerialTx(str .. "\r\n", 80, debug_port)

		return valueShifted
	end

	-- str = "fall through"
	-- ez.SerialTx(str .. "\r\n", 80, debug_port)

	return 0 -- Error
end

-- //Return the average of a given number of readings
-- //Gives up after 1000ms so don't call this function to average 8 samples setup at 1Hz output (requires 8s)
-- int32_t NAU7802::getAverage(uint8_t averageAmount)
-- {
--   long total = 0;
--   uint8_t samplesAquired = 0;

--   unsigned long startTime = millis();
--   while (1)
--   {
--     if (available() == true)
--     {
--       total += getReading();
--       if (++samplesAquired == averageAmount)
--         break; //All done
--     }
--     if (millis() - startTime > 1000)
--       return (0); //Timeout - Bail with error
--     delay(1);
--   }
--   total /= averageAmount;

--   return (total);
-- }

-- //Call when scale is setup, level, at running temperature, with nothing on it
-- void NAU7802::calculateZeroOffset(uint8_t averageAmount)
-- {
--   setZeroOffset(getAverage(averageAmount));
-- }

-- //Sets the internal variable. Useful for users who are loading values from NVM.
-- void NAU7802::setZeroOffset(int32_t newZeroOffset)
-- {
--   _zeroOffset = newZeroOffset;
-- }

-- int32_t NAU7802::getZeroOffset()
-- {
--   return (_zeroOffset);
-- }

-- //Call after zeroing. Provide the float weight sitting on scale. Units do not matter.
-- void NAU7802::calculateCalibrationFactor(float weightOnScale, uint8_t averageAmount)
-- {
--   int32_t onScale = getAverage(averageAmount);
--   float newCalFactor = (onScale - _zeroOffset) / (float)weightOnScale;
--   setCalibrationFactor(newCalFactor);
-- }

-- //Pass a known calibration factor into library. Helpful if users is loading settings from NVM.
-- //If you don't know your cal factor, call setZeroOffset(), then calculateCalibrationFactor() with a known weight
-- void NAU7802::setCalibrationFactor(float newCalFactor)
-- {
--   _calibrationFactor = newCalFactor;
-- }

-- float NAU7802::getCalibrationFactor()
-- {
--   return (_calibrationFactor);
-- }

-- //Returns the y of y = mx + b using the current weight on scale, the cal factor, and the offset.
-- float NAU7802::getWeight(bool allowNegativeWeights, uint8_t samplesToTake)
-- {
--   int32_t onScale = getAverage(samplesToTake);

--   //Prevent the current reading from being less than zero offset
--   //This happens when the scale is zero'd, unloaded, and the load cell reports a value slightly less than zero value
--   //causing the weight to be negative or jump to millions of pounds
--   if (allowNegativeWeights == false)
--   {
--     if (onScale < _zeroOffset)
--       onScale = _zeroOffset; //Force reading to zero
--   }

--   float weight = (onScale - _zeroOffset) / _calibrationFactor;
--   return (weight);
-- }

-- //Set Int pin to be high when data is ready (default)
-- bool NAU7802::setIntPolarityHigh()
-- {
--   return (clearBit(NAU7802_CTRL1_CRP, NAU7802_CTRL1)); //0 = CRDY pin is high active (ready when 1)
-- }

-- //Set Int pin to be low when data is ready
-- bool NAU7802::setIntPolarityLow()
-- {
--   return (setBit(NAU7802_CTRL1_CRP, NAU7802_CTRL1)); //1 = CRDY pin is low active (ready when 0)
-- }

-- //Mask & set a given bit within a register
-- bool NAU7802::setBit(uint8_t bitNumber, uint8_t registerAddress)
-- {
--   uint8_t value = getRegister(registerAddress);
--   value |= (1 << bitNumber); //Set this bit
--   return (setRegister(registerAddress, value));
-- }
function NAU7802_setBit(bitNumber, registerAddress) -- bool NAU7802::setBit(uint8_t bitNumber, uint8_t registerAddress)
	local value, str

	-- str = "setBit(bit=" .. tostring(bitNumber) .. ", reg=" .. tostring(registerAddress) .. ")"
	-- ez.SerialTx(str .. "\r\n", 80, debug_port)

	value = NAU7802_getRegister(registerAddress)

	-- str = "value " .. string.format("%02X", value)
	-- ez.SerialTx(str .. "\r\n", 80, debug_port)

	-- str = "value |= (1 << " .. tostring(bitNumber) .. ")"
	-- ez.SerialTx(str .. "\r\n", 80, debug_port)

	value = value | (1 << bitNumber)	-- Set this bit

	-- str = "value " .. string.format("%02X", value)
	-- ez.SerialTx(str .. "\r\n", 80, debug_port)

	return NAU7802_setRegister(registerAddress, value)
end

-- //Mask & clear a given bit within a register
-- bool NAU7802::clearBit(uint8_t bitNumber, uint8_t registerAddress)
-- {
--   uint8_t value = getRegister(registerAddress);
--   value &= ~(1 << bitNumber); //Set this bit
--   return (setRegister(registerAddress, value));
-- }
function NAU7802_clearBit(bitNumber, registerAddress) -- bool NAU7802::clearBit(uint8_t bitNumber, uint8_t registerAddress)
	local value, str

	-- str = "clearBit(bit=" .. tostring(bitNumber) .. ", reg=" .. tostring(registerAddress) .. ")"
	-- ez.SerialTx(str .. "\r\n", 80, debug_port)

	value = NAU7802_getRegister(registerAddress)
	-- str = "value = " .. string.format("%02X", value)
	-- ez.SerialTx(str .. "\r\n", 80, debug_port)

	-- str = "value &= ~(1 << " .. tostring(bitNumber) .. ")"
	-- ez.SerialTx(str .. "\r\n", 80, debug_port)

	value = value & ((~(1 << bitNumber)) & 0xff)	-- Clear this bit

	-- str = "value = " .. string.format("%02X", value)
	-- ez.SerialTx(str .. "\r\n", 80, debug_port)

	return NAU7802_setRegister(registerAddress, value)
end

-- //Return a given bit within a register
-- bool NAU7802::getBit(uint8_t bitNumber, uint8_t registerAddress)
-- {
--   uint8_t value = getRegister(registerAddress);
--   value &= (1 << bitNumber); //Clear all but this bit
--   return (value);
-- }
function NAU7802_getBit(bitNumber, registerAddress) -- bool NAU7802::getBit(uint8_t bitNumber, uint8_t registerAddress)
	local value, str

	-- str = "getBit(bit=" .. tostring(bitNumber) .. ", reg=" .. tostring(registerAddress) .. ")"
	-- ez.SerialTx(str .. "\r\n", 80, debug_port)

	value = NAU7802_getRegister(registerAddress)

	-- str = "value = " .. string.format("%02X", value)
	-- ez.SerialTx(str .. "\r\n", 80, debug_port)

	-- str = "value &= (1 << " .. tostring(bitNumber) .. ")"
	-- ez.SerialTx(str .. "\r\n", 80, debug_port)

	value = value & ((1 << bitNumber) & 0xff)	-- Clear this bit
	value = value > 0 and true or false

	-- str = "value = " .. tostring(value)
	-- ez.SerialTx(str .. "\r\n", 80, debug_port)

	return value
end

-- //Get contents of a register
-- uint8_t NAU7802::getRegister(uint8_t registerAddress)
-- {
--   _i2cPort->beginTransmission(_deviceAddress);
--   _i2cPort->write(registerAddress);
--   if (_i2cPort->endTransmission() != 0)
--     return (-1); //Sensor did not ACK

--   _i2cPort->requestFrom((uint8_t)_deviceAddress, (uint8_t)1);

--   if (_i2cPort->available())
--     return (_i2cPort->read());

--   return (-1); //Error
-- }
function NAU7802_getRegister(registerAddress)
	local result, str
	result = ez.I2Cread(_deviceAddress,registerAddress)


	if result == nil then
		result = -1
		-- return -1
	else
		result = string.byte(result, 1)
		-- return string.byte(result, 1)
	end

	-- str = "0x" .. string.format("%02X", result) ..  "=getRegister(addr=0x" .. string.format("%02X",_deviceAddress) .. ", reg=0x" .. string.format("%02X",registerAddress) .. ")"
	-- ez.SerialTx(str .. "\r\n", 80, debug_port)

	return result
end

-- //Send a given value to be written to given address
-- //Return true if successful
-- bool NAU7802::setRegister(uint8_t registerAddress, uint8_t value)
-- {
--   _i2cPort->beginTransmission(_deviceAddress);
--   _i2cPort->write(registerAddress);
--   _i2cPort->write(value);
--   if (_i2cPort->endTransmission() != 0)
--     return (false); //Sensor did not ACK
--   return (true);
-- }
function NAU7802_setRegister(registerAddress, value)
	local result

	-- local str
	-- str = "setRegister(addr=0x" .. string.format("%02X",_deviceAddress) .. ", reg=0x" .. string.format("%02X",registerAddress) .. ", val=0x" .. string.format("%02X",value) ..")"
	-- ez.SerialTx(str .. "\r\n", 80, debug_port)

	result = ez.I2Cwrite(_deviceAddress,registerAddress, value)

	-- Multi byte I2C write
	-- data = string.char(value)
	-- result = ez.I2Cwrite(_deviceAddress,registerAddress, data, string.len(data))

	return result
end

function printLine(font_height, line, str) -- Show a title sequence for the program
	local x1, y1, x2, y2, bg
	-- Display Size -> 320x240 

	-- Erase Old Weight
	x1 = 0
	y1 = font_height * line
	x2 = 320
	y2 = font_height * line + font_height

	bg = (8 * line)

	ez.BoxFill(x1,y1, x2,y2, ez.RGB(bg,bg,bg)) -- X, Y, Width, Height, Color

	-- Display Line
	ez.SetColor(ez.RGB(0,0,255))
	ez.SetFtFont(fn, font_height * 0.70) -- Font Number, Height, Width
	ez.SetXY(x1, y1)
	print(str)
	-- ez.Wait_ms(200)
end

function titleScreen(fn) -- Show a title sequence for the program
	ez.Cls(ez.RGB(0,0,0))
	printLine(font_height, 0, "Pull Test - MqpQ")
	ez.Wait_ms(500)
end


function readPin(fn, pin) -- Show a title sequence for the program
	printLine(font_height, 2, string.format("%0.0f", pin ) )
	-- ez.SetXY(x1 + 50, y1)
	-- print(string.format("%0.2f", ez.Pin(pin) ))
end

function display_pause()
	-- for i= 9,0,-1 do
	-- 	printLine(font_height, 1, string.format("%d", i) )
	-- 	ez.Wait_ms(100)
	-- end
	-- ez.Wait_ms(250)
end

debug_port = 0
-- Event Function
function DebugPortReceiveFunction(byte)
	ez.SerialTx(byte, 1, debug_port)
end

fn = 14
font_height = 240 / 8 -- = 30

weight = 0
tare = -37475
weight_max = 0
pin = 0

-- Wait 10 seconds for USB to enumerate
-- ez.Wait_ms(10000)

-- open the RS-232 port
ez.SerialOpen("DebugPortReceiveFunction", debug_port)
ez.SerialTx("Debug Port Open\r\n", 80, debug_port)

-- Main
titleScreen(fn)

ez.SerialTx("ez.I2CopenMaster\r\n", 80, debug_port)
result = ez.I2CopenMaster()
printLine(font_height, 5, "I2C Open: " .. tostring(result) )

ez.SerialTx("NAU7802_isConnected\r\n", 80, debug_port)
result = NAU7802_isConnected()
printLine(font_height, 6, "isConnected:" .. tostring(result) )

ez.SerialTx("NAU7802_begin\r\n", 80, debug_port)
result = NAU7802_begin(true) -- return boolean
printLine(font_height, 6, "begin:" .. tostring(result) )
ez.Wait_ms(1000)


while 1 do
	-- printLine(font_height, 1, tostring(weight) .. " " .. tostring( (~weight) & 0xff ) )
	-- printLine(font_height, 2, tostring(weight) .. " " .. tostring(1 << weight) )
	-- weight = weight + 1

	-- get new weight
	local weight = weight + 10.0001

	if NAU7802_available() == true then
		local weight = NAU7802_getReading() - tare
		printLine(font_height, 1, "Reading: " .. string.format("%d", weight))
	end

	-- readPin(fn, pin)
	-- pin = pin + 1

	-- if pin > 100 then 
	-- 	pin = 0 
	-- end

	-- ez.Wait_ms(25)

end

