/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.util.StringJoiner;

import edu.wpi.first.hal.FRCNetComm.tInstances;
import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.RobotDriveBase;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;

/**
 * A duplication of the WPILib class for driving differential drive/skid-steer drive platforms such as the Kit of Parts drive
 * base, "tank drive", or West Coast Drive.  This will only implement arcade drive and be cusomized to allow for PID velocity
 * control of the drvetrain
 *
 * <p>These drive bases typically have drop-center / skid-steer with 3 wheels per side
 * (e.g., 6WD). This class takes a SpeedController per side. For 
 * six motor drivetrains, construct and pass in {@link edu.wpi.first.wpilibj.SpeedControllerGroup}
 * instances as follows.
 *
 * <p>Six motor drivetrain:
 * <pre><code>
 * public class Robot {
 *   Spark m_frontLeft = new Spark(1);
 *   Spark m_midLeft = new Spark(2);
 *   Spark m_rearLeft = new Spark(3);
 *   SpeedControllerGroup m_left = new SpeedControllerGroup(m_frontLeft, m_midLeft, m_rearLeft);
 *
 *   Spark m_frontRight = new Spark(4);
 *   Spark m_midRight = new Spark(5);
 *   Spark m_rearRight = new Spark(6);
 *   SpeedControllerGroup m_right = new SpeedControllerGroup(m_frontRight, m_midRight, m_rearRight);
 *
 *   DifferentialDrive m_drive = new DifferentialDrive(m_left, m_right);
 * }
 * </code></pre>
 *
 * <p>A differential drive robot has left and right wheels separated by an arbitrary width.
 *
 * <p>Drive base diagram:
 * <pre>
 * |_______|
 * | |   | |
 *   |   |
 * |_|___|_|
 * |       |
 * </pre>
 *
 * <p>Each drive() function provides different inverse kinematic relations for a differential drive
 * robot. Motor outputs for the right side are negated, so motor direction inversion by the user is
 * usually unnecessary.
 *
 * <p>This library uses the NED axes convention (North-East-Down as external reference in the world
 * frame): http://www.nuclearprojects.com/ins/images/axis_big.png.
 *
 * <p>The positive X axis points ahead, the positive Y axis points right, and the positive Z axis
 * points down. Rotations follow the right-hand rule, so clockwise rotation around the Z axis is
 * positive.
 *
 * <p>Inputs smaller then {@value edu.wpi.first.wpilibj.drive.RobotDriveBase#kDefaultDeadband} will
 * be set to 0, and larger values will be scaled so that the full range is still used. This
 * deadband value can be changed with {@link #setDeadband}.
 *
 */

public class DifferentialDrive980 extends RobotDriveBase {
  public static final double kDefaultQuickStopThreshold = 0.2;
  public static final double kDefaultQuickStopAlpha = 0.1;

  private static int instances;

  private final SpeedController m_leftMotor;
  private final SpeedController m_rightMotor;

  private double m_rightSideInvertMultiplier = -1.0;
  private boolean m_reported;

  //PID constants and max velocity measured from the robot
  private static final double LOW_P = 0.05;
  private static final double I = 0.0;
  private static final double LOW_D = 0;
  private static final double MAX_VELOCITY = 17; 

  private static final double HIGH_P = 0.08;
  private static final double HIGH_D = .03; 


  private Encoder leftEncoder; 
  private Encoder rightEncoder; 

  private PIDController leftController;
  private PIDController rightController;

  /**
   * Construct a DifferentialDrive.
   *
   * <p>To pass multiple motors per side, use a {@link SpeedControllerGroup}. If a motor needs to be
   * inverted, do so before passing it in.
   */
  public DifferentialDrive980(SpeedController leftMotor, SpeedController rightMotor , RobotMap robotMap) {
    verify(leftMotor, rightMotor);
    m_leftMotor = leftMotor;
    m_rightMotor = rightMotor;
    addChild(m_leftMotor);
    addChild(m_rightMotor);
    instances++;
    setName("DifferentialDrive", instances);

    //constructing the PID controllers and encoders for feedback
    leftEncoder = robotMap.getLeftDriveEncoder();
    rightEncoder = robotMap.getRightDriveEncoder();
    
    leftEncoder.setPIDSourceType(PIDSourceType.kRate);
    rightEncoder.setPIDSourceType(PIDSourceType.kRate);
    
    leftController = new PIDController(LOW_P, I, LOW_D, leftEncoder, m_leftMotor);
    rightController = new PIDController(LOW_P, I, LOW_D, rightEncoder, m_rightMotor);
    
    
  }

  /**
   * Verifies that all motors are nonnull, throwing a NullPointerException if any of them are.
   * The exception's error message will specify all null motors, e.g. {@code
   * NullPointerException("leftMotor, rightMotor")}, to give as much information as possible to
   * the programmer.
   *
   * @throws NullPointerException if any of the given motors are null
   */
  @SuppressWarnings("PMD.AvoidThrowingNullPointerException")
  private void verify(SpeedController leftMotor, SpeedController rightMotor) {
    if (leftMotor != null && rightMotor != null) {
      return;
    }
    StringJoiner joiner = new StringJoiner(", ");
    if (leftMotor == null) {
      joiner.add("leftMotor");
    }
    if (rightMotor == null) {
      joiner.add("rightMotor");
    }
    throw new NullPointerException(joiner.toString());
  }

  /**
   * Arcade drive method for differential drive platform.
   * The calculated values will be squared to decrease sensitivity at low speeds.
   *
   * @param xSpeed    The robot's speed along the X axis [-1.0..1.0]. Forward is positive.
   * @param zRotation The robot's rotation rate around the Z axis [-1.0..1.0]. Clockwise is
   *                  positive.
   */
  @SuppressWarnings("ParameterName")
  public void arcadeDrive(double xSpeed, double zRotation , boolean enablePID) {
    arcadeDrive(xSpeed, zRotation, true);
  }

  /**
   * Arcade drive method for differential drive platform.
   *
   * @param xSpeed        The robot's speed along the X axis [-1.0..1.0]. Forward is positive.
   * @param zRotation     The robot's rotation rate around the Z axis [-1.0..1.0]. Clockwise is
   *                      positive.
   * @param squareInputs If set, decreases the input sensitivity at low speeds.
   */
  @SuppressWarnings("ParameterName")
  public void arcadeDrive(double xSpeed, double zRotation, boolean squareInputs , boolean enablePID) {
    if (!m_reported) {
      HAL.report(tResourceType.kResourceType_RobotDrive, 2,
                 tInstances.kRobotDrive2_DifferentialArcade);
      m_reported = true;
    }

    xSpeed = limit(xSpeed);
    xSpeed = applyDeadband(xSpeed, m_deadband);

    zRotation = limit(zRotation);
    zRotation = applyDeadband(zRotation, m_deadband);

    // Square the inputs (while preserving the sign) to increase fine control
    // while permitting full power.
    if (squareInputs) {
      xSpeed = Math.copySign(xSpeed * xSpeed, xSpeed);
      zRotation = Math.copySign(zRotation * zRotation, zRotation);
    }

    double leftMotorOutput;
    double rightMotorOutput;

    double maxInput = Math.copySign(Math.max(Math.abs(xSpeed), Math.abs(zRotation)), xSpeed);

    if (xSpeed >= 0.0) {
      // First quadrant, else second quadrant
      if (zRotation >= 0.0) {
        leftMotorOutput = maxInput;
        rightMotorOutput = xSpeed - zRotation;
      } else {
        leftMotorOutput = xSpeed + zRotation;
        rightMotorOutput = maxInput;
      }
    } else {
      // Third quadrant, else fourth quadrant
      if (zRotation >= 0.0) {
        leftMotorOutput = xSpeed + zRotation;
        rightMotorOutput = maxInput;
      } else {
        leftMotorOutput = maxInput;
        rightMotorOutput = xSpeed - zRotation;
      }
    }

    if (enablePID){
      leftController.setSetpoint(limit(leftMotorOutput) * MAX_VELOCITY);
      rightController.setSetpoint(limit(rightMotorOutput) * MAX_VELOCITY * m_rightSideInvertMultiplier);
     
    }
    else{
      m_leftMotor.set(limit(leftMotorOutput) * m_maxOutput);
      m_rightMotor.set(limit(rightMotorOutput) * m_maxOutput * m_rightSideInvertMultiplier);
 
    }

    feed();
  }

  /**
   * Gets if the power sent to the right side of the drivetrain is multipled by -1.
   *
   * @return true if the right side is inverted
   */
  public boolean isRightSideInverted() {
    return m_rightSideInvertMultiplier == -1.0;
  }

  /**
   * Sets if the power sent to the right side of the drivetrain should be multipled by -1.
   *
   * @param rightSideInverted true if right side power should be multipled by -1
   */
  public void setRightSideInverted(boolean rightSideInverted) {
    m_rightSideInvertMultiplier = rightSideInverted ? -1.0 : 1.0;
  }

  @Override
  public void stopMotor() {
    m_leftMotor.stopMotor();
    m_rightMotor.stopMotor();
    feed();
  }

  @Override
  public String getDescription() {
    return "DifferentialDrive";
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.setSmartDashboardType("DifferentialDrive");
    builder.setActuator(true);
    builder.setSafeState(this::stopMotor);
    builder.addDoubleProperty("Left Motor Speed", m_leftMotor::get, m_leftMotor::set);
    builder.addDoubleProperty(
        "Right Motor Speed",
        () -> m_rightMotor.get() * m_rightSideInvertMultiplier,
        x -> m_rightMotor.set(x * m_rightSideInvertMultiplier));
  }

  public void enablePID() {
    leftController.enable();
    rightController.enable();
  }

  public void disablePID() {
    leftController.disable();
    rightController.disable();
  }

  public void setLowP() {
    leftController.setP(LOW_P);
    leftController.setD(LOW_D);
    rightController.setP(LOW_P);
    leftController.setD(LOW_D);
  }

  public void setHighP() {
    leftController.setP(HIGH_P);
    leftController.setD(HIGH_D);
    rightController.setP(HIGH_P);
    rightController.setD(HIGH_D);
  }

  public boolean isPIDEnabled() {
    return leftController.isEnabled() && rightController.isEnabled();
}
}
