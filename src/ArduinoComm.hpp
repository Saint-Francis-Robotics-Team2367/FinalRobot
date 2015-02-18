/*
 * ArduinoComm.hpp
 *
 *  Created on: Feb 16, 2015
 *      Author: daniel
 */

#ifndef ARDUINOCOMM_HPP_
#define ARDUINOCOMM_HPP_

#include "WPILib.h"

#define ARDUINO_ADDR 0xFF

#define RESET 0x00
#define ERR 0x01
#define FULL_PRESSURE 0x02
#define NOT_FULL_PRESSURE 0x03
#define RAINBOW 0x04

class ArduinoComm
{
  public:
    ArduinoComm ();
    void fullPressure();
    void notFullPressure();
    void error();
    void reset();
	void rainbow();
    void preGame(CANTalon *fL,CANTalon *fR,CANTalon *rL,CANTalon *rR,Compressor *comp,PowerDistributionPanel *pdb);
  private:
    I2C *comm;
};

#endif /* ARDUINOCOMM_HPP_ */
