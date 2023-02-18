// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.AutoRoutines;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.DriveStraightOnHeading;
import frc.robot.commands.DriveToAprilTagDistance;

public class Week0_DriveOut extends SequentialCommandGroup {
  /** Creates a new Week0_DriveOut. */
  public Week0_DriveOut() {
    addCommands(new DriveStraightOnHeading(-0.2, 200, 0.0));
    // addCommands(new DriveToAprilTagDistance(-0.2,
    // Constants.ConvertFeetToMeters(10.0)));
  }
}
