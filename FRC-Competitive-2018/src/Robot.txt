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

	//System Constants
	const double THRESHOLD = 0.1;
	
	//Control System
	Joystick *stick0;
	Joystick *stick1;
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
	//DoubleSolenoid clamp;

	AnalogInput LV_MAX_Sonar;


public:
	Robot():
		//Drivetrain
		left0(8),
		left1(10),
		right0(9),
		right1(11),


		//Lift
		lift0(12),
		limTop(6),
		limBottom(7),

		//Intake
		intake0(13),
		intake1(14),

		//Manipulator
	//	clamp(0, 1),

		LV_MAX_Sonar(3)

	{
		stick0 = new Joystick(0);
		stick1 = new Joystick(1);
		m_pdp = new PowerDistributionPanel(),
		encLeft = new Encoder(0, 1, true, Encoder::EncodingType::k4X);
		encRight = new Encoder(2, 3, true, Encoder::EncodingType::k4X);
		encLift = new Encoder(4, 5, true, Encoder::EncodingType::k4X);
	}
	double SonarSensor()
		{
			double supplied_voltage =5;
			double vi= 270/supplied_voltage;
			double distance = LV_MAX_Sonar.GetAverageVoltage() * vi;

		//	printf(" Distance:%f \n", LV_MAX_Sonar.GetVoltage());
			return distance;//measured in inches
		}
	void Encoders()
	{
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
	void RobotInit() {
		left0.SetNeutralMode(NeutralMode::Coast);
		left1.SetNeutralMode(NeutralMode::Coast);
		right0.SetNeutralMode(NeutralMode::Coast);
		right1.SetNeutralMode(NeutralMode::Coast);
		lift0.SetNeutralMode(NeutralMode::Coast);
		intake0.SetNeutralMode(NeutralMode::Coast);
		intake1.SetNeutralMode(NeutralMode::Coast);

		//left1.Set(ControlMode::Follower, 8);
		//right1.Set(ControlMode::Follower, 10);

		left0.SetInverted(true);
		left1.SetInverted(true);

		//left0.ConfigSelectedFeedbackSensor(FeedbackDevice::QuadEncoder, 0, 0);
		//right0.ConfigSelectedFeedbackSensor(FeedbackDevice::QuadEncoder, 0, 0);
	}

	void Autonomous()
	{
		//http://wpilib.screenstepslive.com/s/currentCS/m/getting_started/l/826278-2018-game-data-details
		std::string gameData;
		gameData = frc::DriverStation::GetInstance().GetGameSpecificMessage();

		if(gameData[0] == 'L')  //if the switch we are trying to score in is on the left
		{
			//left auto
		}
		else
		{
			//right auto
		}


	}


	void OperatorControl() override {
		double left;
		double right;

		while(IsEnabled() && IsOperatorControl()){
			float Sonar1 =SonarSensor();
			printf("s: %f \n",Sonar1); //inches

			left = stick0->GetY() - stick0->GetX();
			right = stick0->GetY() + stick0->GetX();

			/*
			if(stick0->GetY() >= THRESHOLD || stick0->GetY() <= -THRESHOLD){
				left0.Set(ControlMode::PercentOutput, stick0->GetY());
				left1.Set(ControlMode::PercentOutput, stick0->GetY());
				right0.Set(ControlMode::PercentOutput, stick0->GetY());
				right1.Set(ControlMode::PercentOutput, stick0->GetY());
			}
			else{
				left0.Set(ControlMode::PercentOutput, 0);
				left1.Set(ControlMode::PercentOutput, 0);

				right0.Set(ControlMode::PercentOutput, 0);
				right1.Set(ControlMode::PercentOutput, 0);
			}
			*/


			if(left >= THRESHOLD || left <= -THRESHOLD){
				left0.Set(ControlMode::PercentOutput, left);
				left1.Set(ControlMode::PercentOutput, left);
			}
			else{
				left0.Set(ControlMode::PercentOutput, 0);
				left1.Set(ControlMode::PercentOutput, 0);
			}

			if(right >= THRESHOLD || right <= -THRESHOLD){
				right0.Set(ControlMode::PercentOutput, right);
				right1.Set(ControlMode::PercentOutput, right);
			}
			else{
				right0.Set(ControlMode::PercentOutput, 0);
				right1.Set(ControlMode::PercentOutput, 0);
			}
			Wait(0.04);

		}
	}


	void Test() override {

	}

private:

};

START_ROBOT_CLASS(Robot)
