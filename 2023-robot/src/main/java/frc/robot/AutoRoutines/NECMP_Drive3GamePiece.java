// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.AutoRoutines;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.DriveArcingTurn;
import frc.robot.commands.DriveStraightOnHeading;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class NECMP_Drive3GamePiece extends SequentialCommandGroup {
  /** Creates a new NECMP_Drive3GamePiece. */
  public NECMP_Drive3GamePiece(int color) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new DriveStraightOnHeading(-0.05, -0.5, 20.0, 0.0 * color));
    addCommands(new DriveStraightOnHeading(-0.5, 120.0, 0.0 * color));
    addCommands(new DriveArcingTurn(-0.5,-0.5, 50, 0.0* color, -75* color));
    addCommands(new DriveArcingTurn(-0.5, -0.2, 15, -75* color, -95* color));
    addCommands(new DriveStraightOnHeading(-0.2, 10.0, -95.0 * color));

    //Come back
    addCommands(new WaitCommand(.1));
    addCommands(new DriveArcingTurn(0.1, 0.5, 20, -95* color, 0* color));
    addCommands(new DriveStraightOnHeading(0.5, 140.0, 5.0 * color));
    addCommands(new DriveStraightOnHeading(0.5, 0.05, 20.0, 5.0 * color));

    // addCommands(new DriveStraightOnHeading(0.5, 30.0, 0.0 * color));
    // addCommands(new DriveStraightOnHeading(0.5, 0.05, 20.0, 0.0 * color));



    // addCommands(new DriveStraightOnHeading(-0.4,-0.3, 60.0, -45.0 * color));

  }
}
