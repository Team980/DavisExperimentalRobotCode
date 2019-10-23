/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.RobotMap;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.RobotPositionalAwarenessEngine;

public class IMUTurn extends Command {
  private Drivetrain drivetrain;
  private RobotPositionalAwarenessEngine posEng;
  private RobotMap robotMap;
  private double angle;
  private double currentAngle;
  private double[] ypr;

  public IMUTurn(Drivetrain drivetrain , RobotMap robotMap , RobotPositionalAwarenessEngine posEng , double angle) {
    this.drivetrain = drivetrain;
    this.posEng = posEng;
    this.robotMap = robotMap;
    this.angle = angle;
    ypr = new double[3];


    requires(drivetrain);
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    robotMap.getImu().setYaw(0);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    robotMap.getImu().getYawPitchRoll(ypr);
    drivetrain.TeleopDrive(0, -2 * (angle - ypr[0]) / angle, false, true);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    if (Math.abs(angle) > Math.abs(ypr[0])){
    return false;
    }
    else{
      return true;
    }
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    robotMap.getImu().getYawPitchRoll(ypr);
    posEng.setFacingAngle(ypr[0]);
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    robotMap.getImu().getYawPitchRoll(ypr);
    posEng.setFacingAngle(ypr[0]);

  }
}
