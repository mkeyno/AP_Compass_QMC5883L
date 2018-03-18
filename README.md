# AP_Compass_QMC5883L
QMC5883L library for using Ardupilot ver 3.2.1  in APM 1.0 hardware 

#installation in Ardupilot

add "#include "AP_Compass_QMC5883L.h" line                              in AP_Compass.h header file
add #define AP_COMPASS_TYPE_QMC5883L  0x06 line                  in Compass.h     header file
add # define MAG_BOARD_ORIENTATION ROTATION_ROLL_180   in Compass.h     header file
add #define HAL_COMPASS_QMC5883L  5                                      in AP_HAL_Boards.h     header file
set  #define CONFIG_COMPASS 	 HAL_Compass_QMC5883L        in config.h      header file
set
  #elif CONFIG_COMPASS == HAL_Compass_QMC5883L   
											static AP_Compass_QMC5883L compass; 
in ArduCopter.pde       header file
