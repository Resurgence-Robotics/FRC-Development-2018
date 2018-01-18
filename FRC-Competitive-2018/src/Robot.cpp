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
#include <ctre/phoenix/MotorControl/ControlMode.h>
#include <ctre/phoenix/MotorControl/NeutralMode.h>
#include <ctre/phoenix/MotorControl/FeedbackDevice.h>

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
		limTop(6),
		limBottom(7),

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
		left0.SetNeutralMode(NeutralMode::Coast);
		left1.SetNeutralMode(NeutralMode::Coast);
		right0.SetNeutralMode(NeutralMode::Coast);
		right1.SetNeutralMode(NeutralMode::Coast);
		lift0.SetNeutralMode(NeutralMode::Coast);
		intake0.SetNeutralMode(NeutralMode::Coast);
		intake1.SetNeutralMode(NeutralMode::Coast);

		//left1.Set(ControlMode::Follower, 8);
		right1.Set(ControlMode::Follower, 10);

		left0.ConfigSelectedFeedbackSensor(FeedbackDevice::QuadEncoder, 0, 0);
		right0.ConfigSelectedFeedbackSensor(FeedbackDevice::QuadEncoder, 0, 0);
	}

	void Autonomous() {
		while(IsEnabled() && IsAutonomous()){
			/*
			printf("\nPreset Encoder value: %i", encLeft->Get());
			encLeft->Reset();
			printf("\nPostset Encoder value: %i", encLeft->Get());
			Wait(1);
			left1.Set(ControlMode::PercentOutput, 0.0);
			printf("\nPost Run Encoder value: %i", encLeft->Get());
			Wait(1);
			*/
			printf("\nPreset Encoder value: %i", left1.GetSensorCollection().GetQuadraturePosition());
			left1.GetSensorCollection().SetQuadraturePosition(0, 1000);
			printf("\nPostset Encoder value: %i", left1.GetSensorCollection().GetQuadraturePosition());
			Wait(1);
			left1.Set(ControlMode::Position, 1250);
			Wait(1);
			printf("\nPost Run Encoder value: %i", left1.GetSensorCollection().GetQuadraturePosition());
			Wait(1);
		}
	}


	void OperatorControl() override {
		while(IsOperatorControl() && IsEnabled()){
			printf("\nIs Lacey useful: %d", limTop.Get());
			Wait(1);

			SmartDashboard::PutNumber("Encoder", left1.GetSensorCollection().GetQuadraturePosition());
		}
	}


	void Test() override {}

private:

};

START_ROBOT_CLASS(Robot)
