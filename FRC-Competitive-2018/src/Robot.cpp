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
	const int LIFT_TOP = 600;
	const int LIFT_BOTTOM = 0;
	const bool TOP = true;
	const bool BOTTOM = false;
	const bool OPEN = true;
	const bool CLOSED = false;
	
	//Control System
	Joystick *stick0;
	Joystick *stick1;
	PowerDistributionPanel* m_pdp;
	AHRS *ahrs;
	DigitalInput posRight;
	DigitalInput posCenter;
	DigitalInput posLeft;
	//AnalogInput LV_MAX_Sonar;

	//Drivetrain
	TalonSRX left0;
	TalonSRX left1;
	TalonSRX right0;
	TalonSRX right1;

	//Lift
	TalonSRX PTO0;
	TalonSRX PTO1;
	Encoder *PTO_Enc;
	DigitalInput limTop;
	DigitalInput limBottom;

	//Intake
	TalonSRX pentacept0;
	TalonSRX pentacept1;
	Solenoid clamp;
	Solenoid pentaTilt;
	DigitalInput boxSensor;

	//LED
	Relay *redLED;
	Relay *greenLED;
	Relay *blueLED;

public:
	Robot():
		//Control System
		//LV_MAX_Sonar(3),
		posRight(1),
		posCenter(2),
		posLeft(3),

		//Drivetrain
		left0(12),
		left1(13),
		right0(4),
		right1(6),

		//Lift
		PTO0(5),
		PTO1(9),
		limTop(6),
		limBottom(7),

		//Intake
		pentacept0(7),
		pentacept1(8),
		clamp(0),
		pentaTilt(1),
		boxSensor(0)
	{
		stick0 = new Joystick(0);
		stick1 = new Joystick(1);
		PTO_Enc = new Encoder(4, 5, true, Encoder::EncodingType::k4X);
		m_pdp = new PowerDistributionPanel();
		ahrs = new AHRS(SerialPort::kMXP);
		redLED = new Relay(0);
		greenLED = new Relay(1);
		blueLED = new Relay(2);
	}

	void RobotInit() {
		left0.SetNeutralMode(NeutralMode::Coast);
		left1.SetNeutralMode(NeutralMode::Coast);
		right0.SetNeutralMode(NeutralMode::Coast);
		right1.SetNeutralMode(NeutralMode::Coast);
		PTO0.SetNeutralMode(NeutralMode::Brake);
		PTO1.SetNeutralMode(NeutralMode::Brake);
		pentacept0.SetNeutralMode(NeutralMode::Coast);
		pentacept1.SetNeutralMode(NeutralMode::Coast);

		left0.SetInverted(true);
		left1.SetInverted(true);
		PTO0.SetInverted(true);
		PTO1.SetInverted(true);
		pentacept1.SetInverted(true);
		
		clamp.Set(false);
		pentaTilt.Set(false);


		left0.ConfigSelectedFeedbackSensor(FeedbackDevice::QuadEncoder, 0, 0);
		left0.SetSensorPhase(true);

		right0.ConfigSelectedFeedbackSensor(FeedbackDevice::QuadEncoder, 0, 0);
		right0.SetSensorPhase(true);
		
		CameraServer::GetInstance()->StartAutomaticCapture();

		printf("Valentina Tereshkova, Reporting for duty.");
	}

	void SetSpeed(double left, double right){
		left0.Set(ControlMode::PercentOutput, left);
		left1.Set(ControlMode::PercentOutput, left);
		right0.Set(ControlMode::PercentOutput, right);
		right1.Set(ControlMode::PercentOutput, right);
		printf("SetSpeed: %f, %f\n", left, right);
		SmartDashboard::PutNumber("SetSpeed Left", left);
		SmartDashboard::PutNumber("SetSpeed Right", right);
	}

	void SetIntakeSpeed(double speed){
		pentacept0.Set(ControlMode::PercentOutput, speed);
		pentacept1.Set(ControlMode::PercentOutput, speed);
		printf("SetSpeed: %f\n", speed);
		SmartDashboard::PutNumber("SetIntakeSpeed", speed);
	}

	void SetLiftSpeed(double speed){
		PTO0.Set(ControlMode::PercentOutput, speed);
		PTO1.Set(ControlMode::PercentOutput, speed);
		printf("SetSpeed: %f\n", speed);
		SmartDashboard::PutNumber("SetLiftSpeed", speed);
	}

	void RunIntakeTime(double speed, double time){
		pentacept0.Set(ControlMode::PercentOutput, speed);
		pentacept1.Set(ControlMode::PercentOutput, speed);
		SmartDashboard::PutNumber("RunIntakeTime", speed);
		Wait(time);
		pentacept0.Set(ControlMode::PercentOutput, 0.0);
		pentacept1.Set(ControlMode::PercentOutput, 0.0);
		SmartDashboard::PutNumber("RunIntakeTime", 0.0);
	}

	void RunLiftTime(double speed, double time){
		PTO0.Set(ControlMode::PercentOutput, speed);
		PTO1.Set(ControlMode::PercentOutput, speed);
		SmartDashboard::PutNumber("RunLiftTime", speed);
		Wait(time);
		PTO0.Set(ControlMode::PercentOutput, 0.0);
		PTO1.Set(ControlMode::PercentOutput, 0.0);
		SmartDashboard::PutNumber("RunLiftTime", 0.0);
	}

	void RunLiftLimits(bool side){
		SmartDashboard::PutBoolean("RunLiftLimits", false);
		if(side){
			while(limTop.Get() && (IsEnabled() && IsAutonomous())){
				PTO0.Set(ControlMode::PercentOutput, 0.7);
				PTO1.Set(ControlMode::PercentOutput, 0.7);
			}
		}
		else{
			while(limBottom.Get() && (IsEnabled() && IsAutonomous())){
				PTO0.Set(ControlMode::PercentOutput, -0.7);
				PTO1.Set(ControlMode::PercentOutput, -0.7);
			}
		}
		PTO0.Set(ControlMode::PercentOutput, 0.0);
		PTO1.Set(ControlMode::PercentOutput, 0.0);
		printf("Lift complete ENC: %i\n", PTO_Enc->Get());
		SmartDashboard::PutBoolean("RunLiftLimits", true);
	}

	void RunLiftPosition(double position){
		PTO_Enc->Reset();

		double wheelRadius = 2.5;
		double wheelCircumpfrence = 2 * 3.14159265 * wheelRadius; //13.8
		double PPR = 1440; //tried 831
		double encIn = PPR / wheelCircumpfrence; //296.8
		double EncTarget = position * encIn; //(60*296.8)=17,808
		printf("EncTarget: %f \n", EncTarget);  //printing out 17776 :)
		printf("encIn:%f \n", encIn);

		if(position > 0){
			while(PTO_Enc->Get() < EncTarget && (IsEnabled() && IsAutonomous())){
				SetLiftSpeed(0.7);
				printf("While Enc: %i\n", PTO_Enc->Get());
			}
		}
		else if(position < 0){
			while(PTO_Enc->Get() > EncTarget && (IsEnabled() && IsAutonomous())){
				SetLiftSpeed(-0.7);
				printf("While Enc: %i\n", PTO_Enc->Get());
			}
		}
		SetLiftSpeed(0.0);
	}

	void ClampToggle(bool state){
		clamp.Set(state);
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

	void PTurn(int angle){
		ahrs->Reset();
		Wait(1);

		double iterationTime = 0.1;//Time in seconds each iteration of the loop should take
		int timeBuffer = 0;

		double kP = 5;

		double error = angle - ahrs->GetYaw();
		double output;

		while(timeBuffer < 5 && (IsEnabled() && IsAutonomous())){//Need to find a stop condition
			error = angle - ahrs->GetYaw();//Error = Final - Current

			output = kP * error;//Sum all components together
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

			if(fabs(error) < 3){
				timeBuffer++;
			}
			else{
				timeBuffer = 0;
			}

			SmartDashboard::PutNumber("Proportional", kP * error);
			SmartDashboard::PutNumber("Output", output);
			SmartDashboard::PutNumber("Error", error);
			SmartDashboard::PutNumber("Setpoint", angle);
			SmartDashboard::PutNumber("Current Angle", ahrs->GetYaw());

			Wait(iterationTime);//Wait the iteration time
		}

		SetSpeed(0.0, 0.0);
		printf("PID Complete\n");

	}

	void PIDTurn(int angle, double timeOut){ //Positive number for clockwise, Negative for anti-clockwise
		ahrs->Reset();
		Timer t1;
		t1.Get();
		t1.Reset();
		t1.Start();

		Wait(0.25);


		double errorPrior = 0;//Error from previous cycle starts at 0 since no previous cycle
		double integral = 0;//Integral starts at 0 since that's how integral work
		double derivative = 0;//Derivative technically doesn't need to be instantiated before the loop, I just thought it looked nicer up here
		double iterationTime = 0.1;//Time in seconds each iteration of the loop should take
		int timeBuffer = 0;

		double kP = 4;//Proportional Component's Tunable Value 	-45 = 0.5	-90 = 0.5
		double kI = 0.25;//Integral Component's Tunable Value 		-45 = 0.5	-90 = 1.0
		double kD = 0.1;//Derivative Component's Tunable Value 		-45 = 0	1	-90 = 0.3

		double error = angle - ahrs->GetYaw();
		double output;

		while(timeBuffer < 5 && (IsEnabled() && IsAutonomous()) && t1.Get() < timeOut){//Need to find a stop condition
			error = angle - ahrs->GetYaw();//Error = Final - Current

			printf("Timer %f\n", t1.Get());

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

			if(fabs(error) < 3){
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

	/* Removed: no sonar sensor on bot
	double SonarSensor(){
		double supplied_voltage =5;
		double vi= 270/supplied_voltage;
		double distance = LV_MAX_Sonar.GetAverageVoltage() * vi;

		//printf("\n Distance:%f", distance);
		return distance;
	}

	void DriveSonar(double distance){
		double sonar = SonarSensor();

		while(sonar > distance){
			SetSpeed(0.6, 0.6);
		}
	}
	*/

	void DriveTime(int time, double speed){
		SetSpeed(speed, speed);
		Wait(time);
		SetSpeed(0, 0);
	}

	void DriveWhile(double distance, double speed){
		left0.SetSelectedSensorPosition(0, 0, 0);
		right0.SetSelectedSensorPosition(0, 0, 0);

		double wheelRadius = 2.5;
		double wheelCircumpfrence = 2 * 3.14159265 * wheelRadius; //13.8
		double PPR = 1440; //tried 831
		double encIn = PPR / wheelCircumpfrence; //296.8
		double EncTarget = distance * encIn; //(60*296.8)=17,808
		printf("EncTarget: %f \n", EncTarget);  //printing out 17776 :)
		printf("encIn:%f \n", encIn);

		if(distance > 0){
			while(right0.GetSelectedSensorPosition(0) < EncTarget && (IsEnabled() && IsAutonomous())){
				SetSpeed(-speed, -speed);
				printf("While Enc: %i\n", right0.GetSelectedSensorPosition(0));
			}
		}
		else if(distance < 0){
			while(right0.GetSelectedSensorPosition(0) > EncTarget && (IsEnabled() && IsAutonomous())){
				SetSpeed(speed, speed);
				printf("While Enc: %i\n", right0.GetSelectedSensorPosition(0));
			}
		}
		SetSpeed(0.0, 0.0);
	}

	void DriveStraightWhile(double distance, double speed){
		left0.SetSelectedSensorPosition(0, 0, 0);
		right0.SetSelectedSensorPosition(0, 0, 0);
		ahrs->Reset();

		double wheelRadius = 2.5;
		double wheelCircumpfrence = 2 * 3.14159265 * wheelRadius; //13.8
		double PPR = 1440; //tried 831
		double encIn = PPR / wheelCircumpfrence; //296.8
		double EncTarget = distance * encIn; //(60*296.8)=17,808
		double gyroCorrection = 0;
		double gP = 0.125;
		printf("EncTarget: %f \n", EncTarget);  //printing out 17776 :)
		printf("encIn:%f \n", encIn);

		if(distance > 0){
			while(right0.GetSelectedSensorPosition(0) < EncTarget && (IsEnabled() && IsAutonomous())){
				gyroCorrection = gP * ahrs->GetYaw();
				DriveFRC(-speed, gyroCorrection);
				printf("While Enc: %i\n", right0.GetSelectedSensorPosition(0));
			}
		}
		else if(distance < 0){
			while(right0.GetSelectedSensorPosition(0) > EncTarget && (IsEnabled() && IsAutonomous())){
				gyroCorrection = gP * ahrs->GetYaw();
				DriveFRC(speed, gyroCorrection);
				printf("While Enc: %i\n", right0.GetSelectedSensorPosition(0));
			}
		}
		SetSpeed(0.0, 0.0);
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
		left0.SetSelectedSensorPosition(0, 0, 0);
		right0.SetSelectedSensorPosition(0, 0, 0);
		ahrs->GetYaw();

		double wheelRadius = 2.5;
		double wheelCircumpfrence = 2 * 3.14159265 * wheelRadius; //13.8
		double PPR = 1440; //tried 831
		double encIn = PPR / wheelCircumpfrence; //296.8
		double EncTarget = distance * encIn; //(60*296.8)=17,808
		printf("EncTarget: %f \n", EncTarget);  //printing out 17776 :)
		printf("encIn:%f \n", encIn);

		double integral = 0;
		double kP = 0.125;
		double kI = 0.15;
		double output = 0;
		double iterationTime = 0.1;
		int timeBuffer = 0;

		double error = EncTarget - right0.GetSelectedSensorPosition(0);

		while(timeBuffer < 10 && (IsEnabled() && IsAutonomous())){
			error = EncTarget - right0.GetSelectedSensorPosition(0);

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
		left0.SetSelectedSensorPosition(0, 0, 0);
		right0.SetSelectedSensorPosition(0, 0, 0);
		ahrs->Reset();

		double wheelRadius = 2.5;
		double wheelCircumpfrence = 2 * 3.14159265 * wheelRadius; //13.8
		double PPR = 1000; //tried 831
		double encIn = PPR / wheelCircumpfrence; //296.8
		double EncTarget = distance * encIn; //(60*296.8)=17,808
		printf("EncTarget: %f \n", EncTarget);  //printing out 17776 :)
		printf("encIn:%f \n", encIn);

		double integral = 0;
		double kP = 0.125;
		double kI = 0.15;
		double gP = 0.125;
		double output = 0;
		double iterationTime = 0.1;
		double gyroCorrection = 0;
		int timeBuffer = 0;

		double error = EncTarget - right0.GetSelectedSensorPosition(0);

		while(timeBuffer < 10 && (IsEnabled() && IsAutonomous())){
			error = EncTarget - right0.GetSelectedSensorPosition(0);

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

			printf("Encoder: %i\n", right0.GetSelectedSensorPosition(0));
			printf("Target: %f\n", EncTarget);
			printf("Error: %f\n", error);
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

		left0.SetNeutralMode(NeutralMode::Brake);
		left1.SetNeutralMode(NeutralMode::Brake);
		right0.SetNeutralMode(NeutralMode::Brake);
		right1.SetNeutralMode(NeutralMode::Brake);
		PTO_Enc->Reset();


		int startPos = 0;//0 = Default, 1 = Right, 2 = Center, 3 = Left
		char allianceSwitch = gameData[0];
		char scale = gameData[1];

		if(!posRight.Get()){
			startPos = 1;
		}
		else if(!posCenter.Get()){
			startPos = 2;
		}
		else if(!posLeft.Get()){
			startPos = 3;
		}
		else{
			startPos = 0;
		}

		if(startPos == 1){//Starting: Right
			printf("Right\n");
			if(allianceSwitch == 'L' && scale == 'L'){//If both scale and switch are on the wrong side
				DriveWhile(120, 0.7);//Cross the auto line and stop
			}
			else if(allianceSwitch == 'R' && scale == 'L'){//If switch is right and scale is wrong
				DriveWhile(115, 0.9);//Score Switch
				RunLiftTime(0.8, 1.5);
				PIDTurn(-90, 5.0);
				pentaTilt.Set(true);
				Wait(0.75);
				RunIntakeTime(-1.0, 1);
			}
			else if(allianceSwitch == 'L' && scale == 'R'){//If scale is right and switch is wrong
				DriveWhile(222, 0.9);//Score Scale
				RunLiftLimits(TOP);
				PIDTurn(-90, 5.0);
				pentaTilt.Set(true);
				Wait(0.75);
				RunIntakeTime(-1.0, 1);
			}
			else if(allianceSwitch == 'R' && scale == 'R'){//If both scale and switch are on the right side
				DriveWhile(222, 0.9);//Score Scale
				RunLiftLimits(TOP);
				PIDTurn(-90, 5.0);
				pentaTilt.Set(true);
				Wait(0.75);
				RunIntakeTime(-1.0, 1);
			}
		}
		else if(startPos == 2){//Starting: Center
			printf("Center\n");
			if(allianceSwitch == 'L'){//If switch is left
				DriveWhile(40, 0.7);//Score Switch
				PIDTurn(-90, 5.0);
				DriveWhile(36, 0.7);
				PIDTurn(90, 5.0);
				RunLiftTime(0.8, 2);
				DriveTime(2, 0.5);
				RunIntakeTime(-1.0, 1);
			}
			else if(allianceSwitch == 'R'){//If switch is right
				DriveWhile(40, 0.7);//Score Switch
				PIDTurn(90, 5.0);
				DriveWhile(36, 0.7);
				PIDTurn(-90, 5.0);
				RunLiftTime(0.8, 2);
				DriveTime(2, 0.5);
				RunIntakeTime(-1.0, 1);
			}
		}
		else if(startPos == 3){//Starting: Left
			printf("Left\n");
			if(allianceSwitch == 'R' && scale == 'R'){//If both scale and switch are on the wrong side
				DriveWhile(120, 0.7);//Cross the auto line and stop
			}
			else if(allianceSwitch == 'L' && scale == 'R'){//If switch is right and scale is wrong
				DriveWhile(115, 0.9);//Score Switch
				RunLiftTime(0.8, 1.5);
				PIDTurn(90, 5.0);
				pentaTilt.Set(true);
				Wait(0.75);
				RunIntakeTime(-1.0, 1);
			}
			else if(allianceSwitch == 'R' && scale == 'L'){//If scale is right and switch is wrong
				DriveWhile(222, 0.9);//Score Scale
				RunLiftLimits(TOP);
				PIDTurn(-90, 5.0);
				pentaTilt.Set(true);
				Wait(0.75);
				RunIntakeTime(-1.0, 1);
			}
			else if(allianceSwitch == 'L' && scale == 'L'){//If both scale and switch are on the right side
				DriveWhile(222, 0.9);//Score Scale
				RunLiftLimits(TOP);
				PIDTurn(-90, 5.0);
				pentaTilt.Set(true);
				Wait(0.75);
				RunIntakeTime(-1.0, 1);
			}
		}
		else{
			printf("Default\n");
		}
	}

	void OperatorControl() override{
		double left;
		double right;

		left0.SetSelectedSensorPosition(0, 0, 0);
		right0.SetSelectedSensorPosition(0, 0, 0);
		PTO_Enc->Reset();

		while(IsEnabled() && IsOperatorControl()){
			//Arcade
			left = stick0->GetY() - stick0->GetX();
			right = stick0->GetY() + stick0->GetX();

			//Tank
			//left = stick0->GetY();
			//right = stick1->GetY();

			//Drive Train (driver 1)
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

			//Lift (driver 2)
			if(stick1->GetRawButton(4)){
				/*
				if(PTO_Enc->Get() < 3000){
					SetLiftSpeed(0.5);
				}
				else if(PTO_Enc->Get() > 3000){
					SetLiftSpeed(-0.5);
				}
				else{
					SetLiftSpeed(0.0);
				}
				*/

				double kP = 0.01;

				double error = 3000 - PTO_Enc->Get();

				double output = kP * error;

				SetLiftSpeed(output);

			}
			else if(stick1->GetY() >= THRESHOLD && limTop.Get() && !stick1->GetRawButton(4)){//Should add limits based on mag switch or encoder
				printf("Stick1 Y = %f\n", stick1->GetY());
				PTO0.Set(ControlMode::PercentOutput, stick1->GetY());
				PTO1.Set(ControlMode::PercentOutput, stick1->GetY());
			}
			else if(stick1->GetY() <= -THRESHOLD && limBottom.Get() && !stick1->GetRawButton(4)){//Lower limits
				printf("Stick1 Y = %f\n", stick1->GetY());
				PTO0.Set(ControlMode::PercentOutput, stick1->GetY());
				PTO1.Set(ControlMode::PercentOutput, stick1->GetY());
			}
			else{
				PTO0.Set(ControlMode::PercentOutput, 0.1);
				PTO1.Set(ControlMode::PercentOutput, 0.1);
			}

			//Intake (driver 2)
			if(stick1->GetRawButton(2)){//Button 2
				pentacept0.Set(ControlMode::PercentOutput, -1);
				pentacept1.Set(ControlMode::PercentOutput, -1);
			}
			else if(stick1->GetRawButton(1)){//Trigger button
				pentacept0.Set(ControlMode::PercentOutput, 1);
				pentacept1.Set(ControlMode::PercentOutput, 1);
			}
			else if(stick1->GetRawButton(5)){//Button 2
				pentacept0.Set(ControlMode::PercentOutput, -0.4);
				pentacept1.Set(ControlMode::PercentOutput, -0.4);
			}
			else{
				pentacept0.Set(ControlMode::PercentOutput, 0.2);
				pentacept1.Set(ControlMode::PercentOutput, 0.2);
			}

			if(stick1->GetRawButton(3)){//Button 3
				clamp.Set(true);
				printf("Clamp Open\n");
			}
			else{
				clamp.Set(false);
			}

			if(stick1->GetRawButton(7)){
				pentaTilt.Set(true);
				printf("Tilt Down\n");
			}
			else{
				pentaTilt.Set(false);
			}

			if(stick0->GetRawButton(5)){
				redLED->Set(Relay::Value::kForward);
			}
			else{
				redLED->Set(Relay::Value::kOff);
			}

			if(stick0->GetRawButton(2)){
				greenLED->Set(Relay::Value::kForward);
			}
			else{
				greenLED->Set(Relay::Value::kOff);
			}

			if(stick0->GetRawButton(6)){
				blueLED->Set(Relay::Value::kForward);
			}
			else{
				blueLED->Set(Relay::Value::kOff);
			}


			printf("Left Encoder: %d\n", left0.GetSelectedSensorPosition(0));
			printf("Right Encoder: %d\n", right0.GetSelectedSensorPosition(0));
			printf("PTO Encoder: %d\n", PTO_Enc->Get());
			printf("Top %i, Bottom %i\n", limTop.Get(), limBottom.Get());

			Wait(0.04);
		}
	}


	void Test() override {//OMG 1080!!!1!11! Lacey made me do this

	}

private:

};

START_ROBOT_CLASS(Robot)
