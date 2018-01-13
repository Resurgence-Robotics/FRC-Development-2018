#include <iostream>
#include <string>

#include <Joystick.h>
#include <SampleRobot.h>
#include <SmartDashboard/SendableChooser.h>
#include <SmartDashboard/SmartDashboard.h>
#include <Timer.h>
#include <wpilib.h>
#include <Math.h>
#include <ctre/Phoenix.h>

class Robot : public frc::SampleRobot {

	//Control System
	Joystick stick0;
	Joystick stick1;
	PowerDistributionPanel* m_pdp;

	//Drivetrain
	TalonSRX left0;
	TalonSRX left1;
	TalonSRX right0;
	TalonSRX right1;
	Encoder* encLeft;
	Encoder* encRight;

	//Lift
	TalonSRX lift0;
	DigitalInput limTop;
	DigitalInput limBottom;
	Encoder* encLift;

	//Intake
	TalonSRX intake0;
	TalonSRX intake1;

	//Manipulator
	DoubleSolenoid clamp;


public:
	Robot():
		//Control System
		stick0(0),
		stick1(1),

		//Drivetrain
		left0(8),
		left1(9),
		right0(10),
		right1(11),


		//Lift
		lift0(12),
		limTop(0),
		limBottom(1),

		//Intake
		intake0(13),
		intake1(14),

		//Manipulator
		clamp(0, 1)
	{
		m_pdp = new PowerDistributionPanel(),
		encLeft = new Encoder(0, 1, true, Encoder::EncodingType::k4X);
		encRight = new Encoder(2, 3, true, Encoder::EncodingType::k4X);
		encLift = new Encoder(4, 5, true, Encoder::EncodingType::k4X);
	}

	void RobotInit() {

	}

	void Autonomous() {

	}


	void OperatorControl() override {

	}


	void Test() override {}

private:

};

START_ROBOT_CLASS(Robot)
