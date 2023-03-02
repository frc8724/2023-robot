// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.AutoRoutines.Week0;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.DriveGetLevel;
import frc.robot.commands.DriveStraight;
import frc.robot.commands.DriveStraightOnHeading;
import frc.robot.commands.DriveToAprilTagDistance;
import frc.robot.commands.DriveStraightOnHeading.DistanceUnits;

public class Week0_LevelStation extends SequentialCommandGroup {
  /** Creates a new Week0_LevelStation. */
  public Week0_LevelStation() {
    // drive back to see the april tag
    addCommands(new DriveStraightOnHeading(-0.2, 100.0, 0.0));
    // use the april tag to get the distance
    // addCommands(new DriveToAprilTagDistance(-0.2,
    // Constants.ConvertFeetToMeters(4.0)));
    // stay level!
    addCommands(new DriveGetLevel());
  }
}
