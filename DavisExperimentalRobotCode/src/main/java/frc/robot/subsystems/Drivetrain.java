/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.DifferentialDrive980;
import frc.robot.RobotMap;

/**
 * Add your docs here.
 */
public class Drivetrain extends Subsystem {
  private DifferentialDrive980 drive980;
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  public Drivetrain(RobotMap robotMap){
    drive980 = new DifferentialDrive980(robotMap.getLeftDrive(), robotMap.getRightDrive(), robotMap);
  }

  public void TeleopDrive(double speed , double turn , boolean squaredInputs , boolean enablePID){
    drive980.arcadeDrive(speed, turn , squaredInputs , enablePID);
  }

  public void enablePID() {
    drive980.enablePID();
  }

  public void disablePID() {
    drive980.disablePID();
  }
  

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}
