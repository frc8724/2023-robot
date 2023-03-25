// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.AutoRoutines;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.DriveArcingTurn;
import frc.robot.commands.DriveStraightOnHeading;
import frc.robot.commands.SystemZero;
import frc.robot.commands.DriveStraightOnHeading.DistanceUnits;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Test_Drive extends SequentialCommandGroup {
  /** Creates a new Test_Drive. */
  public Test_Drive() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new SystemZero());
    addCommands(new DriveArcingTurn(0.1, 0.1, 100, 0.0, 0.0));

    addCommands(new DriveArcingTurn(0.1, 0.1, 100, 0.0, -90.0));
    addCommands(new DriveArcingTurn(0.1, 0.1, 100, -90.0, 0.0));
    // addCommands(new DriveArcingTurn(0.1, 0.1, 100, 0.0, 0.0));

  }
}
