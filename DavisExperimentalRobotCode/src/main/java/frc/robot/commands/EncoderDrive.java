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

public class EncoderDrive extends Command {
  private Drivetrain drivetrain;
  private RobotPositionalAwarenessEngine posEng;
  
  private RobotMap robotMap;
  private double distance;
  private double speedL;
  private double speedR;
  private double lastValueL;
  private double lastValueR;
  private int failsafeCountdown;
  private boolean emergencyEncoderFailure;
  private double[] ypr;


  public EncoderDrive(Drivetrain drivetrain , RobotMap robotMap , RobotPositionalAwarenessEngine posEng ,  double distance) {
    this.drivetrain = drivetrain;
    this.posEng = posEng;
    this.robotMap = robotMap;
    this.distance = distance;
    speedL = 0.0;
    speedR = 0.0;
    lastValueL = 0;
    lastValueR = 0;
    failsafeCountdown = 3;
    emergencyEncoderFailure = false;
    ypr = new double[3];

    requires(drivetrain);
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    //reset encoders
    robotMap.getLeftDriveEncoder().reset();
    robotMap.getRightDriveEncoder().reset();
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    robotMap.getImu().getYawPitchRoll(ypr);
    speedL = 2 * (distance - robotMap.getLeftDriveEncoder().getDistance()) / distance;
    speedR = 2 * (distance - robotMap.getRightDriveEncoder().getDistance()) / distance;
    if (speedL <= speedR){
      drivetrain.TeleopDrive(speedL, -ypr[0] / 45, false, true);
    }
    else{
      drivetrain.TeleopDrive(speedR,  -ypr[0] / 45, false, true);
    }

    
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    if ((distance - robotMap.getLeftDriveEncoder().getDistance()) < 0 || (distance - robotMap.getRightDriveEncoder().getDistance()) < 0 ){
      return true;
    }
    if (robotMap.getLeftDriveEncoder().getDistance() == lastValueL && (robotMap.getRightDriveEncoder().getDistance() == lastValueR )){
      failsafeCountdown--;
      if (failsafeCountdown == 0){
        emergencyEncoderFailure = true;
        return true;
      }  
    }
    else{
      lastValueL = robotMap.getLeftDriveEncoder().getDistance();
      lastValueR = robotMap.getRightDriveEncoder().getDistance();
    }

    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    posEng.setEncoderFailure(emergencyEncoderFailure);
    if (robotMap.getLeftDriveEncoder().getDistance() >= robotMap.getRightDriveEncoder().getDistance()){
      posEng.calculateNewPosition(robotMap.getLeftDriveEncoder().getDistance());
    }
    else{
      posEng.calculateNewPosition(robotMap.getRightDriveEncoder().getDistance());
    }
    drivetrain.TeleopDrive(0, 0, true, !emergencyEncoderFailure);
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
