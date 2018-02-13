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


class Robot : public frc::SampleRobot {

	//System Constants
	const double THRESHOLD = 0.1;
	
	//Control System
	Joystick *stick0;
	Joystick *stick1;
	PowerDistributionPanel* m_pdp;
	AHRS *ahrs;
	AnalogInput LV_MAX_Sonar;

	//Drivetrain
	TalonSRX left0;
	TalonSRX left1;
	TalonSRX right0;
	TalonSRX right1;

	//Lift
	TalonSRX lift0;
	DigitalInput limTop;
	DigitalInput limBottom;

	//Intake
	TalonSRX intake0;
	TalonSRX intake1;

	//Manipulator
	DoubleSolenoid clamp;


public:
	Robot():
		//Control System
		LV_MAX_Sonar(3),

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
		clamp(0, 1)
	{
		stick0 = new Joystick(0);
		stick1 = new Joystick(1);
		m_pdp = new PowerDistributionPanel(),
		ahrs = new AHRS(SerialPort::kMXP);
	}

	void RobotInit() {
		left0.SetNeutralMode(NeutralMode::Coast);
		left1.SetNeutralMode(NeutralMode::Coast);
		right0.SetNeutralMode(NeutralMode::Coast);
		right1.SetNeutralMode(NeutralMode::Coast);
		lift0.SetNeutralMode(NeutralMode::Coast);
		intake0.SetNeutralMode(NeutralMode::Coast);
		intake1.SetNeutralMode(NeutralMode::Coast);


		left0.ConfigSelectedFeedbackSensor(FeedbackDevice::QuadEncoder, 0, 0);
		left0.SetSensorPhase(false);

		right0.ConfigSelectedFeedbackSensor(FeedbackDevice::QuadEncoder, 0, 0);
		right0.SetSensorPhase(false);
	}

	void SetSpeed(double left, double right){
		left0.Set(ControlMode::PercentOutput, left);
		left1.Set(ControlMode::PercentOutput, left);
		right0.Set(ControlMode::PercentOutput, right);
		right1.Set(ControlMode::PercentOutput, right);
		printf("SetSpeed: %f, %f\n", left, right);
	}

	double Map(double x, double in_min, double in_max, double out_min, double out_max){//This function scales one value to a set range
		return ((x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min);
	}

	//The reference I used for PID: http://robotsforroboticists.com/pid-control/
	void PIDTurn45(int angle){ //Positive number for clockwise, Negative for anti-clockwise
		ahrs->Reset();
		Wait(1);

		angle = (angle > 0) ? -45 : 45;

		double errorPrior = 0;//Error from previous cycle starts at 0 since no previous cycle
		double integral = 0;//Integral starts at 0 since that's how integral work
		double derivative = 0;//Derivative technically doesn't need to be instantiated before the loop, I just thought it looked nicer up here
		double iterationTime = 0.1;//Time in seconds each iteration of the loop should take
		int timeBuffer = 0;

		double kP = 0.6;//Proportional Component's Tunable Value 	-45 = 0.5	-90 = 0.5
		double kI = 0.2;//Integral Component's Tunable Value 		-45 = 0.5	-90 = 1.0
		double kD = 0.1;//Derivative Component's Tunable Value 		-45 = 0	1	-90 = 0.3

		double error = angle - ahrs->GetYaw();
		double output;

		while(timeBuffer < 10 && (IsEnabled() && IsAutonomous())){//Need to find a stop condition
			error = angle - ahrs->GetYaw();//Error = Final - Current

			integral = integral + (error*iterationTime);//Integral is summing the value of all previous errors to eliminate steady state error
			derivative = (error - errorPrior)/iterationTime;//Derivative checks the instantaneous velocity of the error to increase stability

			output = (kP * error) + (kI * integral) + (kD * derivative);//Sum all components together
			output = Map(output, -angle, angle, -0.7, 0.7);

			if(angle < 0){
				SetSpeed(output, -output);
			}
			else if(angle > 0){
				SetSpeed(-output, output);
			}
			else{
				printf("Angle = 0");
			}

			if(fabs(error) < 1){
				timeBuffer++;
			}
			else{
				timeBuffer = 0;
			}

			errorPrior = error;//Set previous error to this iterations error for next time

			SmartDashboard::PutNumber("Proportional", kP * error);
			SmartDashboard::PutNumber("Integral", integral);
			SmartDashboard::PutNumber("Derivative", derivative);
			SmartDashboard::PutNumber("Output", output);
			SmartDashboard::PutNumber("Error", error);
			SmartDashboard::PutNumber("Setpoint", angle);
			SmartDashboard::PutNumber("Current Angle", ahrs->GetYaw());

			Wait(iterationTime);//Wait the iteration time
		}

		SetSpeed(0.0, 0.0);

		printf("PID Complete\n");
	}

	void PIDTurn90(int angle){ //Positive number for clockwise, Negative for anti-clockwise
		ahrs->Reset();
		Wait(1);

		angle = (angle > 0) ? -90 : 90;

		double errorPrior = 0;//Error from previous cycle starts at 0 since no previous cycle
		double integral = 0;//Integral starts at 0 since that's how integral work
		double derivative = 0;//Derivative technically doesn't need to be instantiated before the loop, I just thought it looked nicer up here
		double iterationTime = 0.1;//Time in seconds each iteration of the loop should take
		int timeBuffer = 0;

		double kP = 0.6;//Proportional Component's Tunable Value 	-45 = 0.5	-90 = 0.5
		double kI = 0.2;//Integral Component's Tunable Value 		-45 = 0.5	-90 = 1.0
		double kD = 0.1;//Derivative Component's Tunable Value 		-45 = 0	1	-90 = 0.3

		double error = angle - ahrs->GetYaw();
		double output;

		while(timeBuffer < 10 && (IsEnabled() && IsAutonomous())){//Need to find a stop condition
			error = angle - ahrs->GetYaw();//Error = Final - Current

			integral = integral + (error*iterationTime);//Integral is summing the value of all previous errors to eliminate steady state error
			derivative = (error - errorPrior)/iterationTime;//Derivative checks the instantaneous velocity of the error to increase stability

			output = (kP * error) + (kI * integral) + (kD * derivative);//Sum all components together
			output = Map(output, -angle, angle, -0.7, 0.7);

			if(angle < 0){
				SetSpeed(output, -output);
			}
			else if(angle > 0){
				SetSpeed(-output, output);
			}
			else{
				printf("Angle = 0");
			}

			if(fabs(error) < 1){
				timeBuffer++;
			}
			else{
				timeBuffer = 0;
			}

			errorPrior = error;//Set previous error to this iterations error for next time

			SmartDashboard::PutNumber("Proportional", kP * error);
			SmartDashboard::PutNumber("Integral", integral);
			SmartDashboard::PutNumber("Derivative", derivative);
			SmartDashboard::PutNumber("Output", output);
			SmartDashboard::PutNumber("Error", error);
			SmartDashboard::PutNumber("Setpoint", angle);
			SmartDashboard::PutNumber("Current Angle", ahrs->GetYaw());

			Wait(iterationTime);//Wait the iteration time
		}

		SetSpeed(0.0, 0.0);

		printf("PID Complete\n");
	}

	void PIDTurn(int angle){ //Positive number for clockwise, Negative for anti-clockwise
		ahrs->Reset();
		Wait(1);

		double errorPrior = 0;//Error from previous cycle starts at 0 since no previous cycle
		double integral = 0;//Integral starts at 0 since that's how integral work
		double derivative = 0;//Derivative technically doesn't need to be instantiated before the loop, I just thought it looked nicer up here
		double iterationTime = 0.1;//Time in seconds each iteration of the loop should take
		int timeBuffer = 0;

		double kP = 0.6;//Proportional Component's Tunable Value 	-45 = 0.5	-90 = 0.5
		double kI = 0.2;//Integral Component's Tunable Value 		-45 = 0.5	-90 = 1.0
		double kD = 0.1;//Derivative Component's Tunable Value 		-45 = 0	1	-90 = 0.3

		double error = angle - ahrs->GetYaw();
		double output;

		while(timeBuffer < 10 && (IsEnabled() && IsAutonomous())){//Need to find a stop condition
			error = angle - ahrs->GetYaw();//Error = Final - Current

			integral = integral + (error*iterationTime);//Integral is summing the value of all previous errors to eliminate steady state error
			derivative = (error - errorPrior)/iterationTime;//Derivative checks the instantaneous velocity of the error to increase stability

			output = (kP * error) + (kI * integral) + (kD * derivative);//Sum all components together
			output = Map(output, -angle, angle, -0.7, 0.7);

			if(angle < 0){
				SetSpeed(output, -output);
			}
			else if(angle > 0){
				SetSpeed(-output, output);
			}
			else{
				printf("Angle = 0");
			}

			if(fabs(error) < 1){
				timeBuffer++;
			}
			else{
				timeBuffer = 0;
			}

			errorPrior = error;//Set previous error to this iterations error for next time

			SmartDashboard::PutNumber("Proportional", kP * error);
			SmartDashboard::PutNumber("Integral", integral);
			SmartDashboard::PutNumber("Derivative", derivative);
			SmartDashboard::PutNumber("Output", output);
			SmartDashboard::PutNumber("Error", error);
			SmartDashboard::PutNumber("Setpoint", angle);
			SmartDashboard::PutNumber("Current Angle", ahrs->GetYaw());

			Wait(iterationTime);//Wait the iteration time
		}

		SetSpeed(0.0, 0.0);

		printf("PID Complete\n");
	}

	void PIDTuner(){
		ahrs->Reset();
		Wait(3);


		double errorPrior = 0;//Error from previous cycle starts at 0 since no previous cycle
		double integral = 0;//Integral starts at 0 since that's how integral work
		double derivative = 0;//Derivative technically doesn't need to be instantiated before the loop, I just thought it looked nicer up here
		double iterationTime = 0.1;//Time in seconds each iteration of the loop should take

		double kP = 0;//Proportional Component's Tunable Value 	-45 = 0.5	-90 = 0.5
		double kI = 0;//Integral Component's Tunable Value 		-45 = 0.5	-90 = 1.0
		double kD = 0;//Derivative Component's Tunable Value 		-45 = 0	1	-90 = 0.3
		double angle = 0;
		bool hold = true;

		SmartDashboard::PutNumber("kP", kP);
		SmartDashboard::PutNumber("kI", kI);
		SmartDashboard::PutNumber("kD", kD);
		SmartDashboard::PutNumber("Angle", angle);
		SmartDashboard::PutBoolean("Hold", hold);

		double output;
		double error;

		while((IsEnabled() && IsAutonomous())){

			kP = SmartDashboard::GetNumber("kP", 0);
			kI = SmartDashboard::GetNumber("kI", 0);
			kD = SmartDashboard::GetNumber("kD", 0);
			angle = SmartDashboard::GetNumber("Angle", 0);
			hold = SmartDashboard::GetBoolean("Hold", true);

			while(hold){
				kP = SmartDashboard::GetNumber("kP", 0);
				kI = SmartDashboard::GetNumber("kI", 0);
				kD = SmartDashboard::GetNumber("kD", 0);
				angle = SmartDashboard::GetNumber("Angle", 0);
				hold = SmartDashboard::GetBoolean("Hold", true);
			}

			error = angle - ahrs->GetYaw();//Error = Final - Current

			//printf("Pre Calculations\n");
			integral = integral + (error*iterationTime);//Integral is summing the value of all previous errors to eliminate steady state error

			derivative = (error - errorPrior)/iterationTime;//Derivative checks the instantaneous velocity of the error to increase stability

			output = (kP * error) + (kI * integral) + (kD * derivative);//Sum all components together

			output = Map(output, -angle, angle, -0.7, 0.7);

			if(angle < 0){
				SetSpeed(output, -output);
			}
			else if(angle > 0){
				SetSpeed(-output, output);
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
			SmartDashboard::PutNumber("Current Angle", ahrs->GetYaw());

			Wait(iterationTime);//Wait the iteration time
		}
	}

	double SonarSensor(){
		double supplied_voltage =5;
		double vi= 270/supplied_voltage;
		double distance = LV_MAX_Sonar.GetAverageVoltage() * vi;

		//printf("\n Distance:%f", distance);
		return distance;
	}

	void DriveFRC(double outputMagnitude, double curve){
		double leftOutput, rightOutput;
		double m_sensitivity = 0.05;
		if (curve < 0){
			double value = log(-curve);
			double ratio = (value - m_sensitivity)/(value + m_sensitivity);

			if (ratio == 0){ ratio =.0000000001;}

			leftOutput = outputMagnitude / ratio;
			rightOutput = outputMagnitude;
		}
		else if (curve > 0){
			double value = log(curve);
			double ratio = (value - m_sensitivity)/(value + m_sensitivity);

			if (ratio == 0){ ratio =.0000000001;}

			leftOutput = outputMagnitude;
			rightOutput = outputMagnitude / ratio;
		}
		else{
			leftOutput = outputMagnitude;
			rightOutput = outputMagnitude;
		}
		SetSpeed(rightOutput, leftOutput);
	}

	void DrivePID(double distance, double speed){//Drives with Encoders in PID loop
		left1.SetSelectedSensorPosition(0, 0, 0);
		right1.SetSelectedSensorPosition(0, 0, 0);
		ahrs->GetYaw();

		double wheelRadius = 2;
		double wheelCircumpfrence = 2 * 3.14159265 * wheelRadius; //13.8
		double PPR = 1440; //tried 831
		double encIn = PPR / wheelCircumpfrence; //296.8
		double EncTarget = distance * encIn; //(60*296.8)=17,808
		printf("EncTarget: %f \n", EncTarget);  //printing out 17776 :)
		printf("encIn:%f \n", encIn);

		double integral = 0;
		double kP = 0.125;
		double kI = 0.05;
		double output = 0;
		double iterationTime = 0.1;
		int timeBuffer = 0;

		double error = EncTarget - right1.GetSelectedSensorPosition(0);

		while(timeBuffer < 10 && (IsEnabled() && IsAutonomous())){
			error = EncTarget - right1.GetSelectedSensorPosition(0);

			integral = integral + (error*iterationTime);

			output = (kP * error) + (kI * integral);

			output = Map(output, -(EncTarget / 10), (EncTarget / 10), -speed, speed);

			SetSpeed(-output, -output);

			if(error > 50 || error < -50){
				timeBuffer = 0;
			}
			else{
				timeBuffer++;
			}


			printf("Time Buffer: %i\n", timeBuffer);
			SmartDashboard::PutNumber("Proportional", kP * error);
			SmartDashboard::PutNumber("Integral", integral);
			SmartDashboard::PutNumber("Output", output);
			SmartDashboard::PutNumber("Error", error);
			SmartDashboard::PutNumber("Setpoint", EncTarget);
			SmartDashboard::PutNumber("Current Encoder", right1.GetSelectedSensorPosition(0));

			Wait(iterationTime);
		}
		printf("Finished PID\n");
		SetSpeed(0.0, 0.0);
	}

	void DriveStraightPID(double distance, double speed){//Drives with Encoders and Gyro in PID loop
		left1.SetSelectedSensorPosition(0, 0, 0);
		right1.SetSelectedSensorPosition(0, 0, 0);
		ahrs->GetYaw();

		double wheelRadius = 2;
		double wheelCircumpfrence = 2 * 3.14159265 * wheelRadius; //13.8
		double PPR = 1440; //tried 831
		double encIn = PPR / wheelCircumpfrence; //296.8
		double EncTarget = distance * encIn; //(60*296.8)=17,808
		printf("EncTarget: %f \n", EncTarget);  //printing out 17776 :)
		printf("encIn:%f \n", encIn);

		double integral = 0;
		double kP = 0.125;
		double kI = 0.05;
		double gP = 0.125;
		double output = 0;
		double iterationTime = 0.1;
		double gyroCorrection = 0;
		int timeBuffer = 0;

		double error = EncTarget - right1.GetSelectedSensorPosition(0);

		while(timeBuffer < 10 && (IsEnabled() && IsAutonomous())){
			error = EncTarget - right1.GetSelectedSensorPosition(0);

			integral = integral + (error*iterationTime);

			output = (kP * error) + (kI * integral);

			output = Map(output, -(EncTarget / 10), (EncTarget / 10), -speed, speed);

			gyroCorrection = gP * ahrs->GetYaw();

			printf("Gyro Correction: %f\n", gyroCorrection);

			DriveFRC(-output, gyroCorrection);

			if(error > 50 || error < -50){
				timeBuffer = 0;
			}
			else{
				timeBuffer++;
			}


			printf("Time Buffer: %i\n", timeBuffer);
			SmartDashboard::PutNumber("Proportional", kP * error);
			SmartDashboard::PutNumber("Integral", integral);
			SmartDashboard::PutNumber("Output", output);
			SmartDashboard::PutNumber("Error", error);
			SmartDashboard::PutNumber("Setpoint", EncTarget);
			SmartDashboard::PutNumber("Current Encoder", right1.GetSelectedSensorPosition(0));

			Wait(iterationTime);
		}
		printf("Finished PID\n");
		SetSpeed(0.0, 0.0);
	}

	void Autonomous(){
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


	void OperatorControl() override{
		double left;
		double right;

		while(IsEnabled() && IsOperatorControl()){
			//Arcade
			left = stick0->GetY() - stick0->GetX();
			right = stick0->GetY() + stick0->GetX();

			//Tank
			//left = stick0->GetY();
			//right = stick1->GetY();


			if(fabs(left) >= THRESHOLD){
				left0.Set(ControlMode::PercentOutput, left);
				left1.Set(ControlMode::PercentOutput, left);
			}
			else{
				left0.Set(ControlMode::PercentOutput, 0);
				left1.Set(ControlMode::PercentOutput, 0);
			}

			if(fabs(right) >= THRESHOLD){
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
