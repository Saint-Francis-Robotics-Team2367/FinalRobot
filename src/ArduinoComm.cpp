/*
 * ArduinoComm.cpp
 *
 *  Created on: Feb 16, 2015
 *      Author: daniel
 */

#include "ArduinoComm.hpp"

ArduinoComm::ArduinoComm()
{
	this->comm = new I2C(I2C::kOnboard, ARDUINO_ADDR);
}
void ArduinoComm::error()
{
	this->comm->Write(ARDUINO_ADDR, ERR);
}

void ArduinoComm::fullPressure()
{
	this->comm->Write(ARDUINO_ADDR, FULL_PRESSURE);
}
void ArduinoComm::notFullPressure()
{
	this->comm->Write(ARDUINO_ADDR, NOT_FULL_PRESSURE);
}
void ArduinoComm::rainbow()
{
	this->comm->Write(ARDUINO_ADDR, RAINBOW);
}
void ArduinoComm::reset()
{
	this->comm->Write(ARDUINO_ADDR, RESET);
}

void ArduinoComm::preGame(CANTalon *fL, CANTalon *fR, CANTalon *rL,
		CANTalon *rR, Compressor *comp, PowerDistributionPanel *pdb)
{
	uint8_t message = 0x80;
	SmartDashboard::PutNumber("TEST", fL->GetBusVoltage());
	if (fL->GetBusVoltage() > 12) message |= 0x01;
	if (fR->GetBusVoltage() > 12) message |= 0x02;
	if (rL->GetBusVoltage() > 12) message |= 0x04;
	if (rR->GetBusVoltage() > 12) message |= 0x08;
	if (comp->GetPressureSwitchValue()) message |= 0x16;
	if (pdb->GetVoltage() > 12.5) message |= 0x32;

	this->comm->Write(ARDUINO_ADDR, message);
}
