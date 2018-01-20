#include <iostream>
#include <string>
#include <chrono>

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
#include <AHRS.h>

typedef std::chrono::high_resolution_clock Clock;

class Ginger : public frc::SampleRobot {
	//System Constants
	const int THRESHOLD = 0.1;

	//Control System
	AHRS *ahrs;

	//Stick
	Joystick *stick0;

	//Motors
	TalonSRX left0;
	TalonSRX left1;
	TalonSRX right0;
	TalonSRX right1;
	TalonSRX climber0;

	//Pneumatics
	DoubleSolenoid clamp;
	DoubleSolenoid manipSecond;
	DoubleSolenoid chute;
	DoubleSolenoid manipFirst;


public:
	Ginger():
		left0(0),
		left1(1),
		right0(2),
		right1(3),
		climber0(4),

		clamp(0, 4),
		manipSecond(1, 5),
		chute(2, 6),
		manipFirst(3, 7)
	{

	}

	void RobotInit() {
		stick0 = new Joystick(0);

		ahrs = new AHRS(SerialPort::kMXP);

		left0.SetInverted(true);
		left1.SetInverted(true);

		clamp.Set(DoubleSolenoid::Value::kOff);
		manipSecond.Set(DoubleSolenoid::Value::kOff);
		chute.Set(DoubleSolenoid::Value::kOff);
		manipFirst.Set(DoubleSolenoid::Value::kOff);
	}

	void drive(double left, double right){
		left0.Set(ControlMode::PercentOutput, left);
		left1.Set(ControlMode::PercentOutput, left);
		right0.Set(ControlMode::PercentOutput, right);
		right1.Set(ControlMode::PercentOutput, right);
	}

	void basicTurn(int angle, int timeOut){//Clockwise is positive, AntiClockwise is negative. Probably pretty crap
		double startPoint = ahrs->GetYaw();
		double setPoint = startPoint + angle;
		double acceptableError = 5;
		double initialDiff = setPoint - startPoint;//Final - initial

		double currentDiff;
		double speed;
		//double leftOut;
		//double rightOut;

		if(angle > 0){
			while(!(ahrs->GetYaw() > setPoint - acceptableError && ahrs->GetYaw() < setPoint + acceptableError) && IsEnabled() && IsAutonomous()){//If the gyro is within acceptable error
				currentDiff = setPoint - ahrs->GetYaw();//Final - Current

				speed = (currentDiff / initialDiff) / 2;

				drive(-speed, speed);

				SmartDashboard::PutNumber("Start point", startPoint);
				SmartDashboard::PutNumber("Set point", setPoint);
				SmartDashboard::PutNumber("Initial Difference", initialDiff);
				SmartDashboard::PutNumber("Current Difference", currentDiff);
				SmartDashboard::PutNumber("Speed", speed);
				SmartDashboard::PutNumber("NavX Yaw", ahrs->GetYaw());
			}
		}
		else if(angle < 0){
			while(!(ahrs->GetYaw() > setPoint - acceptableError && ahrs->GetYaw() < setPoint + acceptableError) && IsEnabled() && IsAutonomous()){//If the gyro is within acceptable error
				currentDiff = setPoint - ahrs->GetYaw();//Final - Current

				speed = (currentDiff / initialDiff) / 2;

				drive(speed, -speed);

				SmartDashboard::PutNumber("Start point", startPoint);
				SmartDashboard::PutNumber("Set point", setPoint);
				SmartDashboard::PutNumber("Initial Difference", initialDiff);
				SmartDashboard::PutNumber("Current Difference", currentDiff);
				SmartDashboard::PutNumber("Speed", speed);
				SmartDashboard::PutNumber("NavX Yaw", ahrs->GetYaw());
			}
		}
		else{
			printf("You broke the turn function somehow");
		}
	}


	void Autonomous() {
		ahrs->Reset();

		Wait(3);

		left0.SetNeutralMode(NeutralMode::Brake);
		left1.SetNeutralMode(NeutralMode::Brake);
		right0.SetNeutralMode(NeutralMode::Brake);
		right1.SetNeutralMode(NeutralMode::Brake);

		basicTurn(90, 0);

	}


	void OperatorControl() override {
		left0.SetNeutralMode(NeutralMode::Coast);
		left1.SetNeutralMode(NeutralMode::Coast);
		right0.SetNeutralMode(NeutralMode::Coast);
		right1.SetNeutralMode(NeutralMode::Coast);

		double left;
		double right;

		while(IsEnabled() && IsOperatorControl()){
			left = stick0->GetY() - stick0->GetX();
			right = stick0->GetY() + stick0->GetX();



			if(abs(left) >= THRESHOLD){
				left0.Set(ControlMode::PercentOutput, left);
				left1.Set(ControlMode::PercentOutput, left);
			}
			else{
				left0.Set(ControlMode::PercentOutput, 0);
				left1.Set(ControlMode::PercentOutput, 0);
			}

			if(abs(right) >= THRESHOLD){
				right0.Set(ControlMode::PercentOutput, right);
				right1.Set(ControlMode::PercentOutput, right);
			}
			else{
				right0.Set(ControlMode::PercentOutput, 0);
				right1.Set(ControlMode::PercentOutput, 0);
			}

			if(stick0->GetTrigger()){
				clamp.Set(DoubleSolenoid::Value::kForward);
			}
			else{
				clamp.Set(DoubleSolenoid::Value::kReverse);
			}

			if(stick0->GetRawButton(2)){
				manipFirst.Set(DoubleSolenoid::Value::kReverse);
				manipSecond.Set(DoubleSolenoid::Value::kReverse);

			}
			else{
				manipFirst.Set(DoubleSolenoid::Value::kForward);
				manipSecond.Set(DoubleSolenoid::Value::kForward);
			}

			if(stick0->GetRawButton(3)){
				chute.Set(DoubleSolenoid::Value::kForward);
			}
			else{
				chute.Set(DoubleSolenoid::Value::kReverse);
			}

			//Rotation
			SmartDashboard::PutNumber("NavX Pitch", ahrs->GetPitch());
			SmartDashboard::PutNumber("NavX Yaw", ahrs->GetYaw());
			SmartDashboard::PutNumber("NavX Roll", ahrs->GetRoll());

			//Velocity
			SmartDashboard::PutNumber("NavX X Velocity", ahrs->GetVelocityX());
			SmartDashboard::PutNumber("NavX Y Velocity", ahrs->GetVelocityY());
			SmartDashboard::PutNumber("NavX Z Velocity", ahrs->GetVelocityZ());

			//Displacement
			SmartDashboard::PutNumber("NavX X Displacement", ahrs->GetDisplacementX());
			SmartDashboard::PutNumber("NavX Y Displacement", ahrs->GetDisplacementY());
			SmartDashboard::PutNumber("NavX Z Displacement", ahrs->GetDisplacementZ());
		}
	}


	void Test() override {}

private:

};

START_ROBOT_CLASS(Ginger)
