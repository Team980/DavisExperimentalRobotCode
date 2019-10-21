/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.CounterBase;

/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */
public class RobotMap {
	// drive stuff
	private Encoder leftDriveEncoder;
	private Encoder rightDriveEncoder;

	private SpeedControllerGroup leftDrive;
	private SpeedControllerGroup rightDrive;

	private Solenoid shifter;

	// end effector
	private SpeedController endEffectorIntakeMotor;

	// lift
	private SpeedController liftMotor;
	private Encoder liftEncoder;

	private static final double ENCODER_DISTANCE_PER_TICK = 2 * Math.PI * (2.0 / 12) / 2048;
	private double liftDistancePerTick = Math.PI * (2.899 / 12) / 2048; // used diameter of 2.899

	// wrist
	// private Potentiometer wristPotentiometer;
	private SpeedController wristMotor;

	// sensors
	private PigeonIMU imu;
	// public Limelight limelight;

	private Solenoid trophyTruckSolenoid;

	RobotMap() {
		// drive stuff
		var leftFront = new WPI_TalonSRX(1);
		var leftBack = new WPI_TalonSRX(2);
		var leftTop = new WPI_TalonSRX(0);
		leftTop.setInverted(true);

  		var rightFront = new WPI_TalonSRX(4);
		var rightBack = new WPI_TalonSRX(5);
		var rightTop = new WPI_TalonSRX(3);
		rightTop.setInverted(true);

		leftDrive = new SpeedControllerGroup(leftFront, leftBack, leftTop);
		rightDrive = new SpeedControllerGroup(rightFront, rightBack, rightTop);
		//rightDrive.setInverted(true);

		leftDriveEncoder = new Encoder(2, 3, false, CounterBase.EncodingType.k4X);
		//(Channel A port, Channel B port, is it inverted true/false, encoder type)
		leftDriveEncoder.setDistancePerPulse(ENCODER_DISTANCE_PER_TICK);

		rightDriveEncoder = new Encoder(4, 5, true, CounterBase.EncodingType.k4X);
		//(Channel A port, Channel B port, is it inverted true/false, encoder type)
		rightDriveEncoder.setDistancePerPulse(ENCODER_DISTANCE_PER_TICK);

		shifter = new Solenoid(0);

		// end effector
		endEffectorIntakeMotor = new WPI_TalonSRX(13);
		 
		// lift
		liftMotor = new WPI_TalonSRX(15);
		liftEncoder = new Encoder(0, 1, false, CounterBase.EncodingType.k4X);
		liftEncoder.setDistancePerPulse(liftDistancePerTick);

		// wrist
		//wristPotentiometer = new Potentiometer(0);
		wristMotor = new WPI_TalonSRX(11);
		//wristMotor.setName("wrist controller");

		// sensors

		
		imu = new PigeonIMU(rightBack);
		//limelight = new Limelight();

		trophyTruckSolenoid = new Solenoid(2);
	}

	public Encoder getLeftDriveEncoder() {
		return this.leftDriveEncoder;
	}

	public Encoder getRightDriveEncoder() {
		return this.rightDriveEncoder;
	}

	public SpeedControllerGroup getLeftDrive() {
		return this.leftDrive;
	}

	public SpeedControllerGroup getRightDrive() {
		return this.rightDrive;
	}

	public Solenoid getShifter() {
		return this.shifter;
	}

	public SpeedController getEndEffectorIntakeMotor() {
		return this.endEffectorIntakeMotor;
	}

	public SpeedController getLiftMotor() {
		return this.liftMotor;
	}

	public Encoder getLiftEncoder() {
		return this.liftEncoder;
	}

	public double getLiftDistancePerTick() {
		return this.liftDistancePerTick;
	}

	public SpeedController getWristMotor() {
		return this.wristMotor;
	}

	public PigeonIMU getImu() {
		return this.imu;
	}

	/*public Limelight getLimelight() {
		return this.limelight;
	}*/

	public Solenoid getTrophyTruckSolenoid() {
		return this.trophyTruckSolenoid;
	}


  // For example to map the left and right motors, you could define the
  // following variables to use with your drivetrain subsystem.
  // public static int leftMotor = 1;
  // public static int rightMotor = 2;

  // If you are using multiple modules, make sure to define both the port
  // number and the module. For example you with a rangefinder:
  // public static int rangefinderPort = 1;
  // public static int rangefinderModule = 1;
}
