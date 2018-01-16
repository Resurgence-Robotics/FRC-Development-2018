/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team1080.robot;

import edu.wpi.first.wpilibj.SampleRobot;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PowerDistributionPanel;


public class Robot extends SampleRobot {
	
	//Control System
	Joystick stick0;
	Joystick stick1;
	PowerDistributionPanel m_pdp;

	//Drivetrain
	TalonSRX left0;
	TalonSRX left1;
	TalonSRX right0;
	TalonSRX right1;
	Encoder encLeft;
	Encoder encRight;

	//Lift
	TalonSRX lift0;
	DigitalInput limTop;
	DigitalInput limBottom;
	Encoder encLift;

	//Intake
	TalonSRX intake0;
	TalonSRX intake1;

	//Manipulator
	DoubleSolenoid clamp;
	
	
	public Robot() {
		//Control System
		stick0 = new Joystick(0);
		stick1 = new Joystick(1);
		m_pdp = new PowerDistributionPanel();

		//Drivetrain
		left0 = new TalonSRX(8);
		left1 = new TalonSRX(9);
		right0 = new TalonSRX(10);
		right1 = new TalonSRX(11);
		encLeft = new Encoder(0, 1);
		encRight = new Encoder(2, 3);

		//Lift
		lift0 = new TalonSRX(12);
		limTop = new DigitalInput(6);
		limBottom = new DigitalInput(7);
		encLift = new Encoder(4, 5);;

		//Intake
		intake0 = new TalonSRX(13);
		intake1 = new TalonSRX(14);

		//Manipulator
		clamp = new DoubleSolenoid(0, 1);
	}

	@Override
	public void robotInit() {
		
	}

	@Override
	public void autonomous() {
		while(isEnabled() && isAutonomous()) {
			left0.set(ControlMode.Velocity, 1.0);
		}
	}

	@Override
	public void operatorControl() {
		
	}

	@Override
	public void test() {
		
	}
}
