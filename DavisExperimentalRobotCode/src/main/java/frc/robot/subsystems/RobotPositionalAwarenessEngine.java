/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;

/**
 * Add your docs here.
 */
public class RobotPositionalAwarenessEngine extends Subsystem {
  private RobotMap robotMap;

  private double xPosition;
  private double yPosition;
  private double facingAngle;

  private boolean encoderFailure;

  public RobotPositionalAwarenessEngine(RobotMap robotMap){
    this.robotMap = robotMap;
    
    xPosition = 0;
    yPosition = 0;
    facingAngle = 0;

    encoderFailure = false;
  }

  public void calculateNewPosition(double distance){
    xPosition = -distance * Math.sin(facingAngle);
    yPosition = distance * Math.cos(facingAngle);
  }

  public void setFacingAngle(double newAngle){
    facingAngle += dToR(newAngle);
  }

  public double dToR(double degrees){
    return degrees * Math.PI / 180;
  }

  public void setEncoderFailure(boolean failed){
    encoderFailure = failed;
  }

  public boolean getEncoderFailure(){
    return encoderFailure;
  }
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}
