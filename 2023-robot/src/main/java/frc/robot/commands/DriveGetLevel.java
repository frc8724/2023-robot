// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class DriveGetLevel extends CommandBase {
  static final double START_DRIVING_PITCH = 5.0;
  static final double STOP_DRIVING_PITCH = 3.0;
  static final double DRIVE_POWER = 0.1;
  static final double PEAK_PITCH = 0.5;
  static final double SLOP = 2;


  /** Creates a new DriveGetLevel. */
  public DriveGetLevel() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    RobotContainer.drive.setBrake(true);
  }

  double lastPitch = 0.0;
  boolean climbing = false;

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    var pitch = RobotContainer.drive.getPitch();
    // if we are pitched up...
    if (pitch > 0+SLOP) {
      if (climbing) {
        if (pitch < lastPitch  -PEAK_PITCH) {
          climbing = false;
          RobotContainer.drive.speedRacerDrive(0, 0, true);
        }
      } else {
        // if it is really steep, drive forward
        if (pitch > START_DRIVING_PITCH) {
          climbing = true;
          RobotContainer.drive.speedRacerDrive(0.1, 0, true);
        }
      }
    } else if (pitch<0-SLOP){ // else we are pitched down...
      if (climbing) {
        if (pitch > lastPitch + PEAK_PITCH) {
          climbing = false;
          RobotContainer.drive.speedRacerDrive(0, 0, true);
        }
      } else {
        if (pitch < -START_DRIVING_PITCH) {
          climbing = true;
          RobotContainer.drive.speedRacerDrive(-0.1, 0, true);
        }
      }
    }else{
      RobotContainer.drive.speedRacerDrive(0.0);
    }

    lastPitch = pitch;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.drive.speedRacerDrive(0, 0, true);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
