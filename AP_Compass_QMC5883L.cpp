/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
 

/*
 *       AP_Compass_QMC5883L.cpp - Arduino Library for QMC5883L I2C magnetometer
 *       Code by Keyno. DIYDrones.com
 *
 *       Sensor is conected to I2C port
 *       Sensor is initialized in Continuos mode (10Hz)
 *QMC5883L
 */

// AVR LibC Includes
#include <AP_Math.h>
#include <AP_HAL.h>

#include "AP_Compass_QMC5883L.h"

extern const AP_HAL::HAL& hal;

//00000_OutOfRange_DataSkip_DataReady   0x06
#define COMPASS_ADDRESS      0x0D
//BandWideFilte__FeildRange__OutputDataRate__Mode
//   OSR            RNG         ODR           MOD  
//(512,256,128,64) (2G,8G) (10,50,100,200) (standby, continous)
#define ConfigRegA           0x09
//SoftReset_RollOver00000InterruptEnabaling
//Soft_RST ROl_PNT         INT_ENB              
#define ConfigRegB           0x0A
#define Mode_Standby    0x00
#define Mode_Continuous 0x01

#define BaseReg         0b00011101 ///0b00010101
                        
#define ODR_10Hz        0b00000000
#define ODR_50Hz        0b00000100
#define ODR_100Hz       0b00001000
#define ODR_200Hz       0b00001100

#define RNG_2G          0b00000000
#define RNG_8G          0b00010000

#define OSR_512         0b00000000
#define OSR_256         0b01000000
#define OSR_128         0b10000000
#define OSR_64          0b11000000

// read_register - read a register value
bool AP_Compass_QMC5883L::read_register(uint8_t address, uint8_t *value)
{
    if (hal.i2c->readRegister((uint8_t)COMPASS_ADDRESS, address, value) != 0) {
		_healthy[0] = false; 
	//	hal.console->printf_P(PSTR("fail read %X\n"), address);
		return false; }
	//	hal.console->printf_P(PSTR("suc read %X -> %X\n"), address,value);
    return true;
}

// write_register - update a register value
bool AP_Compass_QMC5883L::write_register(uint8_t address, uint8_t value)
{
    if (hal.i2c->writeRegister((uint8_t)COMPASS_ADDRESS, address, value) != 0) {
        _healthy[0] = false;
        return false;
		// hal.console->printf_P(PSTR("fail write %X\n"), address);
    }
	// hal.console->printf_P(PSTR("suc write %X\n"), address);
    return true;
}

// Read Sensor data
bool AP_Compass_QMC5883L::read_raw()
{
   
	uint8_t buff[6];
               //readRegisters(uint8_t addr,   reg,   len, uint8_t* data);
    if (hal.i2c->readRegisters(COMPASS_ADDRESS, 0x00, 6, buff) != 0) 
	{
        if (_healthy[0]) hal.i2c->setHighSpeed(false); 
            _healthy[0] = false;
        _i2c_sem->give();
        return false;
    }	

    int16_t rx, ry, rz;
    rx = (((int16_t)buff[1]) << 8) | buff[0];
    ry = (((int16_t)buff[3]) << 8) | buff[2];
    rz = (((int16_t)buff[5]) << 8) | buff[4];
	
   // if (rx == -4096 || ry == -4096 || rz == -4096)  return false;  // no valid data available
	
    _mag_x = -rx;
    _mag_y =  ry;
    _mag_z = -rz;
 //hal.console->printf_P(PSTR("MagX: %d MagY: %d MagZ: %d\n"), (int)_mag_x, (int)_mag_y, (int)_mag_z);
    return true;
}


// accumulate a reading from the magnetometer
void AP_Compass_QMC5883L::accumulate(void)
{
    if (!_initialised)  return; 
        // someone has tried to enable a compass for the first time
        // mid-flight .... we can't do that yet (especially as we won't have the right orientation!)

   uint32_t tnow = hal.scheduler->micros();
   if (_healthy[0] && _accum_count != 0 && (tnow - _last_accum_time) < 13333)   return;   // the compass gets new data at 75Hz

   if (!_i2c_sem->take(1)) return;   // the bus is busy - try again later
      
   bool result = read_raw();
   _i2c_sem->give();

   if (result) 
   {
	  // the _mag_N values are in the range -2048 to 2047, so we can
	  // accumulate up to 15 of them in an int16_t. Let's make it 14
	  // for ease of calculation. We expect to do reads at 10Hz, and
	  // we get new data at most 75Hz, so we don't expect to
	  // accumulate more than 8 before a read
	  _mag_x_accum += _mag_x;
	  _mag_y_accum += _mag_y;
	  _mag_z_accum += _mag_z;
	  _accum_count++;
	  if (_accum_count == 14) {
								 _mag_x_accum /= 2;
								 _mag_y_accum /= 2;
								 _mag_z_accum /= 2;
								 _accum_count = 7;
							  }
	  _last_accum_time = tnow;
   }
}


/*
 *  re-initialise after a IO error
 */
bool AP_Compass_QMC5883L::re_initialise()
{
  
   if (!write_register(ConfigRegA, _base_config) )     return false;
   // hal.console->printf_P(PSTR("re_initialise true\n"));
    return true;
}


// Public Methods //////////////////////////////////////////////////////////////
bool AP_Compass_QMC5883L::init()
{
    
	int numAttempts = 0, good_count = 0;
    bool success = false;
    
    uint16_t expected_x = 715;
    uint16_t expected_yz = 715;
    float gain_multiple = 1.0;
	
  if (!write_register(ConfigRegB,0x80)) return false; //softReset  must reset first         
  hal.scheduler->delay(10);
       write_register(0x0B, 0x01);//SET/RESET Period
  
    _i2c_sem = hal.i2c->get_semaphore();
    if (!_i2c_sem->take(HAL_SEMAPHORE_BLOCK_FOREVER))  hal.scheduler->panic(PSTR("Failed to get QMC5883L semaphore")); 
    // determine if we are using 5843 or 5883L
    _base_config = BaseReg;
    if (  !write_register(ConfigRegA, Mode_Continuous | ODR_200Hz | Mode_Continuous| RNG_8G | OSR_512 ) 
	//	|| !read_register(ConfigRegA, &_base_config)  
	) 
	{
        _healthy[0] = false;
        _i2c_sem->give();
        return false;
    }
	
    
        product_id = AP_COMPASS_TYPE_QMC5883L;
       
        /*
          note that the HMC5883 datasheet gives the x and y expected
          values as 766 and the z as 713. Experiments have shown the x
          axis is around 766, and the y and z closer to 713.
         */
        expected_x = 766;
        expected_yz  = 713;
        gain_multiple = 660.0 / 1090;  // adjustment for runtime vs calibration gain
    

    calibration[0] = 0;
    calibration[1] = 0;
    calibration[2] = 0;

    while ( success == 0 && numAttempts < 25 && good_count < 5)
    {
        // record number of attempts at initialisation
        numAttempts++;
        // read values from the compass
        hal.scheduler->delay(50);
        if (!read_raw())  continue; // we didn't read valid values            

        hal.scheduler->delay(10);

        float cal[3];

        //   hal.console->printf_P(PSTR("%d) mag: %d - %d - %d \n"),numAttempts, _mag_x, _mag_y, _mag_z);
        cal[0] = fabsf(expected_x / (float)_mag_x);
        cal[1] = fabsf(expected_yz / (float)_mag_y);
        cal[2] = fabsf(expected_yz / (float)_mag_z);

        //  hal.console->printf_P(PSTR(" cal=%.2f   %.2f   %.2f\n"),numAttempts, cal[0], cal[1], cal[2]);

        // we throw away the first two samples as the compass may
        // still be changing its state from the application of the
        // strap excitation. After that we accept values in a
        // reasonable range
        if (numAttempts > 2 &&
            cal[0] > 0.7f && cal[0] < 1.35f &&
            cal[1] > 0.7f && cal[1] < 1.35f &&
            cal[2] > 0.7f && cal[2] < 1.35f) 
		{
            //  hal.console->printf_P(PSTR("  good\n") );
            good_count++;
            calibration[0] += cal[0];
            calibration[1] += cal[1];
            calibration[2] += cal[2];
        }

#if 0
        /* useful for debugging */
        hal.console->printf_P(PSTR("MagX: %d MagY: %d MagZ: %d\n"), (int)_mag_x, (int)_mag_y, (int)_mag_z);
        hal.console->printf_P(PSTR("CalX: %.2f CalY: %.2f CalZ: %.2f\n"), cal[0], cal[1], cal[2]);
#endif
    }

	//hal.console->printf_P(PSTR("good_count: %d  \n"),good_count);
    if (good_count >= 5)
						{
							/*
							  The use of gain_multiple below is incorrect, as the gain
							  difference between 2.5Ga mode and 1Ga mode is already taken
							  into account by the expected_x and expected_yz values.  We
							  are not going to fix it however as it would mean all
							  APM1/APM2 users redoing their compass calibration. The
							  impact is that the values we report on APM1/APM2 are lower
							  than they should be (by a multiple of about 0.6). This
							  doesn't have any impact other than the learned compass
							  offsets
							 */
							calibration[0] = calibration[0] * gain_multiple / good_count;
							calibration[1] = calibration[1] * gain_multiple / good_count;
							calibration[2] = calibration[2] * gain_multiple / good_count;
							success = true;
						} 
	else //always run 
			{
				/* best guess */
				calibration[0] = 1.0;
				calibration[1] = 1.0;
				calibration[2] = 1.0;
			}

    // leave test mode
    if (!re_initialise()) 
	{
        _i2c_sem->give();
        return false;
    }

    _i2c_sem->give();
    _initialised = true;

	// perform an initial read
	_healthy[0] = true;
	read();

#if 0
    hal.console->printf_P(PSTR("CalX: %.2f CalY: %.2f CalZ: %.2f\n"), calibration[0], calibration[1], calibration[2]);
#endif

    return true;//always success;
}

// Read Sensor data
bool AP_Compass_QMC5883L::read()
{
    if (!_initialised) return false;
        // someone has tried to enable a compass for the first time
        // mid-flight .... we can't do that yet (especially as we won't have the right orientation!)
        //

    if (!_healthy[0]) 
	{
        if (hal.scheduler->millis() < _retry_time) return false; 
        if (!re_initialise()) 
		{
            _retry_time = hal.scheduler->millis() + 1000;
			hal.i2c->setHighSpeed(false);
            return false;
        }
    }

	if (_accum_count == 0) 
	{
	   accumulate();///---> read_raw();
	   if (!_healthy[0] || _accum_count == 0)
	   {
		  // try again in 1 second, and set I2c clock speed slower
		  _retry_time = hal.scheduler->millis() + 1000;
		  hal.i2c->setHighSpeed(false);
		  return false;
	   }
	}

	_field[0].x = _mag_x_accum * calibration[0] / _accum_count;
	_field[0].y = _mag_y_accum * calibration[1] / _accum_count;
	_field[0].z = _mag_z_accum * calibration[2] / _accum_count;
	_accum_count = 0;
	_mag_x_accum = _mag_y_accum = _mag_z_accum = 0;

    last_update = hal.scheduler->micros(); // record time of update

    
    if (product_id == AP_COMPASS_TYPE_QMC5883L) _field[0].rotate(ROTATION_YAW_90);  // rotate to the desired orientation
    // apply default board orientation for this compass type. This is a noop on most boards
     _field[0].rotate(MAG_BOARD_ORIENTATION);   
     _field[0].rotate((enum Rotation)_orientation.get()); // add user selectable orientation

    if (!_external)  _field[0].rotate(_board_orientation); // and add in AHRS_ORIENTATION setting if not an external compass   
       //Vector3f _field[COMPASS_MAX_INSTANCES]; 
    _field[0] += _offset[0].get(); //AP_Vector3f _offset[COMPASS_MAX_INSTANCES];

    // apply motor compensation Vector3f    _motor_offset[COMPASS_MAX_INSTANCES]; AP_Vector3f _motor_compensation[COMPASS_MAX_INSTANCES];
    if(_motor_comp_type != AP_COMPASS_MOT_COMP_DISABLED && _thr_or_curr != 0.0f) 
	{
        _motor_offset[0] = _motor_compensation[0].get() * _thr_or_curr;
        _field[0] += _motor_offset[0];
    }
	else  		  _motor_offset[0].zero(); 

    _healthy[0] = true;

    return true;
}
