/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.InstantCommand;
import frc.robot.subsystems.Drivetrain;

/**
 * Add your docs here.
 */
public class EnablePIDDriveCommand extends InstantCommand {
  private Drivetrain drivetrain;
  private boolean enablePID;
  /**
   * Add your docs here.
   */
  public EnablePIDDriveCommand(boolean enablePID , Drivetrain drivetrain) {
    super();
    this.drivetrain = drivetrain;
    this.enablePID = enablePID;
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
  }

  // Called once when the command executes
  @Override
  protected void initialize() {
    if (enablePID){
      drivetrain.enablePID();
    }
    else{
      drivetrain.disablePID();
    }
  }

}
