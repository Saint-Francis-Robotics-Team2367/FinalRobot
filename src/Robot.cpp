#include "WPILib.h"
#include "ArduinoComm.hpp"

//MOT IDS
#define FL_ID 3
#define FR_ID 2
#define RL_ID 1
#define RR_ID 4

//RELAY IDS
#define FAN_ID 0

//SOLENOID IDS
#define LIFTER_FCHAN 1
#define LIFTER_RCHAN 0

#define FGATE_FCHAN 2
#define FGATE_RCHAN 3

#define GATE_FCHAN 4
#define GATE_RCHAN 5

#define HOPPER_FCHAN 6
#define HOPPER_RCHAN 7

//DELAYS
#define ARM_DELAY 2
#define GATE_DELAY 3
#define GATE_CLOSING_DELAY 3
#define HOPPER_FORWARD_DELAY 2
#define FGATE_OPEN_DELAY 2

//States
#define RESET_STATE 0
#define RAISING_TOTE 1
#define CLOSING_GATE 2
#define ULOAD_LIFTER_DOWN 3
#define FGATE_OPENNING 4

//PNEUMATIC POS
#define HOPPER_FORWARD DoubleSolenoid::kForward
#define HOPPER_BACK DoubleSolenoid::kReverse

#define LIFTER_UP DoubleSolenoid::kForward
#define LIFTER_DOWN DoubleSolenoid::kReverse

#define FGATE_OPEN DoubleSolenoid::kForward
#define FGATE_CLOSE DoubleSolenoid::kReverse

#define GATE_OPEN DoubleSolenoid::kForward
#define GATE_CLOSE DoubleSolenoid::kReverse

//MACROS
#define CURR_TIME Timer::GetFPGATimestamp()

/**
 * This is a demo program showing how to use Mecanum control with the RobotDrive class.
 */
class Robot: public SampleRobot
{

	// Channels for the wheels
	CANTalon *frontLeftMot;
	CANTalon *frontRightMot;
	CANTalon *rearLeftMot;
	CANTalon *rearRightMot;
	RobotDrive *robotDrive;	// robot drive system
	Joystick *firstDriver;			// only joystick
	Joystick *secondDriver;
	Compressor *comp;
	PowerDistributionPanel *pdb;
	ArduinoComm *leds;
	Relay *fan;

	DoubleSolenoid *lifter;
	DoubleSolenoid *frontGate;
	DoubleSolenoid *gate;
	DoubleSolenoid *hopper;

	IMAQdxSession session;
	Image *frame;
	IMAQdxError imaqError;

public:
	Robot()					// as they are declared above.
	{
		this->frontLeftMot = new CANTalon(FL_ID);
		this->frontRightMot = new CANTalon(FR_ID);
		this->rearLeftMot = new CANTalon(RL_ID);
		this->rearRightMot = new CANTalon(RR_ID);

		this->comp = new Compressor();
		this->pdb = new PowerDistributionPanel();
		this->leds = new ArduinoComm();

		this->robotDrive = new RobotDrive(this->frontLeftMot, this->rearLeftMot,
				this->frontRightMot, this->rearRightMot);
		this->robotDrive->SetExpiration(0.1);
		this->robotDrive->SetInvertedMotor(RobotDrive::kFrontLeftMotor, true);// invert the left side motors
		this->robotDrive->SetInvertedMotor(RobotDrive::kRearLeftMotor, true);// you may need to change or remove this to match your robot
		this->fan = new Relay(FAN_ID, Relay::kForwardOnly);

		this->lifter = new DoubleSolenoid(LIFTER_FCHAN, LIFTER_RCHAN);
		this->frontGate = new DoubleSolenoid(FGATE_FCHAN, FGATE_RCHAN);
		this->gate = new DoubleSolenoid(GATE_FCHAN, GATE_RCHAN);
		this->hopper = new DoubleSolenoid(HOPPER_FCHAN, HOPPER_RCHAN);

		this->firstDriver = new Joystick(0);
		this->secondDriver = new Joystick(1);

		/////////////////////////////////////////////////////////////////////////////////////

		frame = imaqCreateImage(IMAQ_IMAGE_RGB, 0);
		//the camera name (ex "cam0") can be found through the roborio web interface
		imaqError = IMAQdxOpenCamera("cam0", IMAQdxCameraControlModeController,
				&session);
		if (imaqError != IMAQdxErrorSuccess)
		{
			DriverStation::ReportError(
					"IMAQdxOpenCamera error: "
							+ std::to_string((long) imaqError) + "\n");
		}
		imaqError = IMAQdxConfigureGrab(session);
		if (imaqError != IMAQdxErrorSuccess)
		{
			DriverStation::ReportError(
					"IMAQdxConfigureGrab error: "
							+ std::to_string((long) imaqError) + "\n");
		}
		//////////////////////////////////////////////////////////////////////////////////////
	}

	/**
	 * Runs the motors with Mecanum drive.
	 */
	void OperatorControl()
	{

		this->robotDrive->SetSafetyEnabled(false);
		this->leds->preGame(this->frontLeftMot, this->frontRightMot,
				this->rearLeftMot, this->rearRightMot, this->comp, this->pdb);
		this->comp->Start();
		float lastCompTime = CURR_TIME;
		float lastDriveTime = CURR_TIME;
		float raisingTime = CURR_TIME;
		float gateClosingTime = CURR_TIME;
		float uloadLiftUp = CURR_TIME;
		float gateOpenTime = CURR_TIME;
		float loweringTime = CURR_TIME;

		int state = RESET_STATE;
		IMAQdxStartAcquisition(session);

		while (IsOperatorControl() && IsEnabled())
		{
			//RAISE MAIN PNEUMATIC
			if (this->firstDriver->GetRawButton(6) && state == RESET_STATE)
			{
				this->lifter->Set(LIFTER_UP);
				raisingTime = CURR_TIME;
				state = RAISING_TOTE;
			}
			//OPENNING GATE
			else if(CURR_TIME - raisingTime>ARM_DELAY && state == RAISING_TOTE)
			{
				this->gate->Set(GATE_OPEN);
				gateClosingTime = CURR_TIME;
				state = CLOSING_GATE;
			}
			//CLOSING GATE AND LOWER LIFTER
			else if(CURR_TIME - gateClosingTime>GATE_DELAY && state == CLOSING_GATE)
			{
				this->gate->Set(GATE_CLOSE);
				state = RESET_STATE;
				this->lifter->Set(LIFTER_DOWN);
			}


			//RAISE LIFTER
			if(this->firstDriver->GetRawButton(5) && state == RESET_STATE)
			{
				this->lifter->Set(LIFTER_UP);
				uloadLiftUp = CURR_TIME;
				state = ULOAD_LIFTER_DOWN;
			}
			//OPEN GATE
			else if(CURR_TIME -uloadLiftUp >HOPPER_FORWARD_DELAY && state == ULOAD_LIFTER_DOWN)
			{
				this->gate->Set(GATE_OPEN);
				gateOpenTime = CURR_TIME;
				state = FGATE_OPENNING;
			}
			//LOWER LIFTER AND OPEN FRONT GATE
			else if(CURR_TIME -gateOpenTime >FGATE_OPEN_DELAY && state == FGATE_OPENNING)
			{
				this->frontGate->Set(FGATE_OPEN);
				this->lifter->Set(LIFTER_DOWN);
				state = RESET_STATE;
			}
			//RESET, CLOSE FRONT GATE AND GATE, PULL HOPPER BACK
			if(this->firstDriver->GetRawButton(1))
			{
				this->frontGate->Set(FGATE_CLOSE);
				this->gate->Set(GATE_CLOSE);
				this->hopper->Set(HOPPER_BACK);
			}
			/*/////////////////////////////////////////////////////////////////////////////////

			 2nd Driver Manual Control

			 ////////////////////////////////////////////////////////////////////////////////*/
			if (this->secondDriver->GetRawButton(6))					//
			this->lifter->Set(LIFTER_UP);
			else if (this->secondDriver->GetRawButton(5))
			this->lifter->Set(LIFTER_DOWN);

			if(this->secondDriver->GetRawButton(3))
			this->gate->Set(GATE_OPEN);
			else if (this->secondDriver->GetRawButton(2))
			this->gate->Set(GATE_CLOSE);

			if(this->secondDriver->GetRawButton(1))
			this->frontGate->Set(FGATE_OPEN);
			else if(this->secondDriver->GetRawButton(4))
			this->frontGate->Set(FGATE_CLOSE);

			if(this->secondDriver->GetRawButton(7))
			this->frontGate->Set(HOPPER_FORWARD);
			else if(this->secondDriver->GetRawButton(8))
			this->frontGate->Set(HOPPER_BACK);

			/*/////////////////////////////////////////////////////////////////////////////////

			 Fans and LEDs

			 ////////////////////////////////////////////////////////////////////////////////*/
			if (CURR_TIME - lastCompTime > 1)
			{
				if (this->comp->Enabled())
				{
					this->fan->Set(Relay::kForward);
					this->leds->rainbow();
				}
				else
				{
					this->leds->fullPressure();
					this->fan->Set(Relay::kOff);
				}
				lastCompTime = CURR_TIME;
			}
			/*/////////////////////////////////////////////////////////////////////////////////

			 ROBOT DRIVE

			 ////////////////////////////////////////////////////////////////////////////////*/
			if (CURR_TIME - lastDriveTime > 0.005)
			{
				this->robotDrive->MecanumDrive_Cartesian(
						deadZone(firstDriver->GetRawAxis(0) * -1),
						deadZone(firstDriver->GetRawAxis(1) * -1),
						deadZone(firstDriver->GetRawAxis(4) * -1));
				lastDriveTime = CURR_TIME;
			}
			/*/////////////////////////////////////////////////////////////////////////////////

			 Image Processing

			 ////////////////////////////////////////////////////////////////////////////////*/
			IMAQdxGrab(session, frame, true, NULL);
			if (imaqError != IMAQdxErrorSuccess)
			{
				DriverStation::ReportError(
						"IMAQdxGrab error: " + std::to_string((long) imaqError)
						+ "\n");
			}
			else
			{
				CameraServer::GetInstance()->SetImage(frame);
			}
			//////////////////////////////////////////////////////////////////////////////////
			Wait(0.005);// wait 5ms to avoid hogging CPU cycles
		}
		IMAQdxStopAcquisition(session);
		this->leds->reset();

	}
	float deadZone(float x)
	{
		if (std::abs(x) < 0.05)
			return 0;
		else
			return x;
	}

};

START_ROBOT_CLASS(Robot);
