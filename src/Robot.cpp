#include "WPILib.h"
#include "ArduinoComm.hpp"

#define FL_ID 3
#define FR_ID 2
#define RL_ID 1
#define RR_ID 4
#define FAN_ID 0

#define LIFTER_FCHAN 0
#define LIFTER_RCHAN 1

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
    Joystick *stick;			// only joystick
    Compressor *comp;
    PowerDistributionPanel *pdb;
    ArduinoComm *leds;
    Relay *fan;

    DoubleSolenoid *lifter;


public:
	Robot()							// as they are declared above.
	{
		this->frontLeftMot = new CANTalon(FL_ID);
		this->frontRightMot = new CANTalon(FR_ID);
		this->rearLeftMot = new CANTalon(RL_ID);
		this->rearRightMot = new CANTalon(RR_ID);

		this->comp = new Compressor();
		this->pdb = new PowerDistributionPanel();
		this->leds = new ArduinoComm();

		this->robotDrive = new RobotDrive(this->frontLeftMot,this->rearLeftMot,this->frontRightMot,this->rearRightMot);
		this->robotDrive->SetExpiration(0.1);
		this->robotDrive->SetInvertedMotor(RobotDrive::kFrontLeftMotor, true);	// invert the left side motors
		this->robotDrive->SetInvertedMotor(RobotDrive::kRearLeftMotor, true);	// you may need to change or remove this to match your robot
		this->fan = new Relay(FAN_ID,Relay::kForwardOnly);

		this->lifter = new DoubleSolenoid (LIFTER_FCHAN,LIFTER_RCHAN);


		this->stick = new Joystick(0);
	}

	/**
	 * Runs the motors with Mecanum drive.
	 */
	void OperatorControl()
	{

		this->robotDrive->SetSafetyEnabled(false);
		this->leds->preGame(this->frontLeftMot,this->frontRightMot,this->rearLeftMot,this->rearRightMot,this->comp,this->pdb);
		this->comp->Start();
		float currTime = Timer::GetFPGATimestamp();
		float lastCompTime = currTime;
		float lastDriveTime = currTime;

		while (IsOperatorControl() && IsEnabled())
		{
		  currTime = Timer::GetFPGATimestamp();
			if(this->stick->GetRawButton(4))this->lifter->Set(DoubleSolenoid::kForward);
			else if (this->stick->GetRawButton(1)) this->lifter->Set(DoubleSolenoid::kReverse);
			if(currTime-lastCompTime>1)
			{
			  if(this->comp->Enabled())
			    {
			      this->fan->Set(Relay::kForward);
			      this->leds->notFullPressure();
			    }
			  else
			    {
			      this->leds->fullPressure();
			      this->fan->Set(Relay::kOff);
			    }
			  lastCompTime = currTime;
			}
			if(currTime-lastDriveTime>0.005)
			{
			  this->robotDrive->MecanumDrive_Cartesian(deadZone(stick->GetRawAxis(0)*-1),deadZone(stick->GetRawAxis(1)*-1), deadZone(stick->GetRawAxis(4)*-1));
			  lastDriveTime = currTime;
			}
			Wait(0.001); // wait 5ms to avoid hogging CPU cycles
		}
	}
	float deadZone(float x)
	{
	  if(std::abs(x)<0.05)return 0;
	  else return x;
	}

};

START_ROBOT_CLASS(Robot);
