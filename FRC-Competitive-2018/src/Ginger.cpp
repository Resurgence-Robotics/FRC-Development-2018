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
#include <AHRS.h>

class Ginger : public frc::SampleRobot {
	//System Constants
	const int THRESHOLD = 0.05;

	//Control System
	AHRS *ahrs;

	//Stick
	Joystick *stick0;
	Joystick *stick1;

	//Motors
	TalonSRX left0;
	TalonSRX left1;
	TalonSRX right0;
	TalonSRX right1;
	TalonSRX climber0;

	//Added so David can use ginger as a test-bed
	TalonSRX david0;
	TalonSRX david1;

	//Pneumatics
	DoubleSolenoid clamp;
	DoubleSolenoid manipSecond;
	DoubleSolenoid chute;
	DoubleSolenoid manipFirst;

	AnalogInput LV_MAX_Sonar; //sonar sensor


public:
	Ginger():
		left0(0),
		left1(1),
		right0(2),
		right1(3),
		climber0(4),

		david0(4),
		david1(7),

		clamp(0, 4),
		manipSecond(1, 5),
		chute(2, 6),
		manipFirst(3, 7),

		LV_MAX_Sonar(3)
	{

	}

	void RobotInit() {
		stick0 = new Joystick(0);
		stick1 = new Joystick(1);

		ahrs = new AHRS(SerialPort::kMXP);

		//https://github.com/CrossTheRoadElec/Phoenix-Examples-Languages/blob/master/C%2B%2B/SetSensorPosition/src/Robot.cpp
		//first # is pidIdx(0 for primary closed loop and 1 for cascaded closed loop), second # is timeout in ms
		left0.SetInverted(true);
		left1.SetInverted(true);
		left1.ConfigSelectedFeedbackSensor(phoenix::motorcontrol::FeedbackDevice::QuadEncoder, 0, 10);
		left1.SetSensorPhase(false);

		right1.ConfigSelectedFeedbackSensor(phoenix::motorcontrol::FeedbackDevice::QuadEncoder, 0, 10);
		right1.SetSensorPhase(false);

		clamp.Set(DoubleSolenoid::Value::kOff);
		manipSecond.Set(DoubleSolenoid::Value::kOff);
		chute.Set(DoubleSolenoid::Value::kOff);
		manipFirst.Set(DoubleSolenoid::Value::kOff);
	}

	void Drive(double left, double right){
		left0.Set(ControlMode::PercentOutput, left);
		left1.Set(ControlMode::PercentOutput, left);
		right0.Set(ControlMode::PercentOutput, right);
		right1.Set(ControlMode::PercentOutput, right);
	}

	//The reference I used: http://robotsforroboticists.com/pid-control/
	void PIDTurn45(int angle){ //Positive number for clockwise, Negative for anti-clockwise
		ahrs->Reset();
		Wait(3);

		angle = (angle > 0) ? -45 : 45;

		double errorPrior = 0;//Error from previous cycle starts at 0 since no previous cycle
		double integral = 0;//Integral starts at 0 since that's how integral work
		double derivative = 0;//Derivative technically doesn't need to be instantiated before the loop, I just thought it looked nicer up here
		double iterationTime = 0.1;//Time in seconds each iteration of the loop should take

		double kP = 0.5;//Proportional Component's Tunable Value 	-45 = 0.5	-90 = 0.5
		double kI = 0.5;//Integral Component's Tunable Value 		-45 = 0.5	-90 = 1.0
		double kD = 0.1;//Derivative Component's Tunable Value 		-45 = 0	1	-90 = 0.3

		double error = angle - ahrs->GetYaw();
		double output;

		//printf("Pre Loop\n");

		while((error > 1 || error < -1) && (IsEnabled() && IsAutonomous())){//Need to find a stop condition
			error = angle - ahrs->GetYaw();//Error = Final - Current

			//printf("Pre Calculations\n");

			integral = integral + (error*iterationTime);//Integral is summing the value of all previous errors to eliminate steady state error

			derivative = (error - errorPrior)/iterationTime;//Derivative checks the instantaneous velocity of the error to increase stability

			output = (kP * error) + (kI * integral) + (kD * derivative);//Sum all components together

			output = Map(output, -angle, angle, -0.7, 0.7);

			if(angle < 0){
				Drive(output, -output);
			}
			else if(angle > 0){
				Drive(-output, output);
			}
			else{
				printf("Angle = 0");
			}

			errorPrior = error;//Set previous error to this iterations error for next time

			//printf("Pre Print\n");

			SmartDashboard::PutNumber("Proportional", kP * error);
			SmartDashboard::PutNumber("Integral", integral);
			SmartDashboard::PutNumber("Derivative", derivative);
			SmartDashboard::PutNumber("Output", output);
			SmartDashboard::PutNumber("Error", error);
			SmartDashboard::PutNumber("Setpoint", angle);
			SmartDashboard::PutNumber("Current Angle", ahrs->GetYaw());

			Wait(iterationTime);//Wait the iteration time
		}

		Drive(0.0, 0.0);

		printf("PID Complete\n");
	}

	void PIDTurn90(int angle){ //Positive number for clockwise, Negative for anti-clockwise
		ahrs->Reset();
		Wait(3);

		angle = (angle > 0) ? -90 : 90;

		double errorPrior = 0;//Error from previous cycle starts at 0 since no previous cycle
		double integral = 0;//Integral starts at 0 since that's how integral work
		double derivative = 0;//Derivative technically doesn't need to be instantiated before the loop, I just thought it looked nicer up here
		double iterationTime = 0.1;//Time in seconds each iteration of the loop should take

		double kP = 0.5;//Proportional Component's Tunable Value 	-45 = 0.5	-90 = 0.5
		double kI = 1.0;//Integral Component's Tunable Value 		-45 = 0.5	-90 = 1.0
		double kD = 0.3;//Derivative Component's Tunable Value 		-45 = 0	1	-90 = 0.3

		double error = angle - ahrs->GetYaw();
		double output;

		while((error > 1 || error < -1) && (IsEnabled() && IsAutonomous())){//Need to find a stop condition
			error = angle - ahrs->GetYaw();//Error = Final - Current

			//printf("Pre Calculations\n");
			integral = integral + (error*iterationTime);//Integral is summing the value of all previous errors to eliminate steady state error

			derivative = (error - errorPrior)/iterationTime;//Derivative checks the instantaneous velocity of the error to increase stability

			output = (kP * error) + (kI * integral) + (kD * derivative);//Sum all components together

			output = Map(output, -angle, angle, -0.7, 0.7);

			if(angle < 0){
				Drive(output, -output);
			}
			else if(angle > 0){
				Drive(-output, output);
			}
			else{
				printf("Angle = 0");
			}

			errorPrior = error;//Set previous error to this iterations error for next time

			//printf("Pre Print\n");

			SmartDashboard::PutNumber("Proportional", kP * error);
			SmartDashboard::PutNumber("Integral", integral);
			SmartDashboard::PutNumber("Derivative", derivative);
			SmartDashboard::PutNumber("Output", output);
			SmartDashboard::PutNumber("Error", error);
			SmartDashboard::PutNumber("Setpoint", angle);
			SmartDashboard::PutNumber("Current Angle", ahrs->GetYaw());

			Wait(iterationTime);//Wait the iteration time
		}

	Drive(0.0, 0.0);

	printf("PID Complete\n");
	}

	double SonarSensor(){
		double supplied_voltage =5;
		double vi= 270/supplied_voltage;
		double distance = LV_MAX_Sonar.GetAverageVoltage() * vi;

		//printf("\n Distance:%f", distance);
		return distance;
	}


	void DriveFRC(float outputMagnitude, float curve){
		float leftOutput, rightOutput;
		float m_sensitivity = 0.5;
		if (curve < 0){
			float value = log(-curve);
			float ratio = (value - m_sensitivity)/(value + m_sensitivity);

			if (ratio == 0){ ratio =.0000000001;}

			leftOutput = outputMagnitude / ratio;
			rightOutput = outputMagnitude;
		}
		else if (curve > 0){
			float value = log(curve);
			float ratio = (value - m_sensitivity)/(value + m_sensitivity);

			if (ratio == 0){ ratio =.0000000001;}

			leftOutput = outputMagnitude;
			rightOutput = outputMagnitude / ratio;
		}
		else{
			leftOutput = outputMagnitude;
			rightOutput = outputMagnitude;
		}
		Drive(rightOutput, leftOutput);
	}

	void DriveWithEnc(float target, float speed){
		left1.SetSelectedSensorPosition(0, 0, 0);
		right1.SetSelectedSensorPosition(0, 0, 0);//reset encoders
		ahrs->Reset();
		Wait(0.25);

		float enc = 0;
		float kp = 0.125;
		if(target > 0){//positive/forward
			while((target > enc) && (IsAutonomous()) && (IsEnabled())){
				enc = left1.GetSelectedSensorPosition(0); //set enc to value of encoder
				printf("enc:%f \n", enc);
				float correction = kp * ahrs->GetYaw();
				printf("gyroAngle:%f \n", ahrs->GetYaw());
				DriveFRC(-speed, correction); //negative is forwards
				Wait(0.01);
				printf("going forward \n");
			}
		}
		else if (target < 0){//negative/backwards
			while((target < enc) && (IsAutonomous()) && (IsEnabled())){
				enc = left1.GetSelectedSensorPosition(0); //set enc to value of encoder
				printf("enc:%f \n", enc);
				float correction = kp * ahrs->GetYaw();
				printf("gyroAngle:%f \n", ahrs->GetYaw());
				DriveFRC(speed, correction); //negative is forwards
				Wait(0.01);
				printf("going backwards \n");
			}
		}
		else{
			printf("Target = 0");
		}
		Drive(0.0, 0.0);
		printf("outside of if statements \n");
	}

	void EncoderDrive(float distance){//tested (about 1/2 inches short)
		float wheelRadius= 2.2;
		float wheelCircumpfrence = 2* 3.142 * wheelRadius; //13.8
		float PPR = 831; //tried 831 @29.5  was 4096
		float encIn= PPR/wheelCircumpfrence; //296.8
		printf("encIn:%f \n", encIn);
		float EncTarget= distance*encIn; //(60*296.8)=17,808
		printf("EncTarget: %f \n", EncTarget);  //printing out 17776 :)
		DriveWithEnc(EncTarget, 0.25);
		//29.5/831=30/x
	}

	double Map(double x, double in_min, double in_max, double out_min, double out_max){//This function scales one value to a set range
		return ((x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min);
	}

	void Autonomous() {
		left0.SetNeutralMode(NeutralMode::Brake);
		left1.SetNeutralMode(NeutralMode::Brake);
		right0.SetNeutralMode(NeutralMode::Brake);
		right1.SetNeutralMode(NeutralMode::Brake);

		EncoderDrive(30);




	/*	PIDTurn45(1);
		Wait(3);
		//PIDTurn(90);
		Wait(3);
		//PIDTurn(45);
	*/
	}


	void OperatorControl() override {
		ahrs->Reset();
		left1.SetSelectedSensorPosition(0, 0, 0);
		right1.SetSelectedSensorPosition(0, 0, 0);

		left0.SetNeutralMode(NeutralMode::Coast);
		left1.SetNeutralMode(NeutralMode::Coast);
		right0.SetNeutralMode(NeutralMode::Coast);
		right1.SetNeutralMode(NeutralMode::Coast);

		double left;
		double right;

		while(IsEnabled() && IsOperatorControl()){
			left = stick0->GetY() - stick0->GetX();
			right = stick0->GetY() + stick0->GetX();

			SmartDashboard::PutNumber("Left Output", left);
			SmartDashboard::PutNumber("Right Output", right);

			SmartDashboard::PutNumber("Stick 0 Y", stick0->GetY());
			SmartDashboard::PutNumber("Stick 0 X", stick0->GetX());

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


			if(stick0->GetTrigger()){
				clamp.Set(DoubleSolenoid::Value::kForward);
			}
			else{
				clamp.Set(DoubleSolenoid::Value::kReverse);
			}

			manipFirst.Set(DoubleSolenoid::Value::kForward);
			manipSecond.Set(DoubleSolenoid::Value::kForward);



			/*
			if(stick0->GetRawButton(2)){
				manipFirst.Set(DoubleSolenoid::Value::kReverse);
				manipSecond.Set(DoubleSolenoid::Value::kReverse);

			}
			else{
				manipFirst.Set(DoubleSolenoid::Value::kForward);
				manipSecond.Set(DoubleSolenoid::Value::kForward);
			}
			*/

			if(stick0->GetRawButton(3)){
				chute.Set(DoubleSolenoid::Value::kForward);
			}
			else{
				chute.Set(DoubleSolenoid::Value::kReverse);
			}

			if(abs(stick1->GetY()) > THRESHOLD){
				david0.Set(ControlMode::PercentOutput, stick1->GetY());
				david1.Set(ControlMode::PercentOutput, -stick1->GetY());
			}
			else{
				david0.Set(ControlMode::PercentOutput, 0);
				david1.Set(ControlMode::PercentOutput, 0);
			}

			float Sonar1 =SonarSensor();
			printf("s: %f \n",Sonar1);


			SmartDashboard::PutNumber("Left Encoder", left1.GetSelectedSensorPosition(0));
			SmartDashboard::PutNumber("Left Velocity", left1.GetSelectedSensorVelocity(0));
			SmartDashboard::PutNumber("Right Encoder", right1.GetSelectedSensorPosition(0));
			SmartDashboard::PutNumber("Right Velocity", right1.GetSelectedSensorVelocity(0));


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
