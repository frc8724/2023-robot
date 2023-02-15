// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.AutoRoutines;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.ShoulderGoto;
import frc.robot.commands.TrajectoryCommand;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Test_1 extends SequentialCommandGroup {
  final String path1 = "C:\\Users\\kippy\\OneDrive\\Documents\\GitHub\\2023-robot\\2023-robot\\src\\main\\deploy\\pathplanner\\generatedJSON\\New Path.wpilib.json";

  /** Creates a new Test_1. */
  public Test_1() {
    // Test auto to raise shoulder, drive a path, lower shoulder, drive a path
    addCommands(new ShoulderGoto(10.0),
        TrajectoryCommand.loadPathplannerCommand(path1, true));
    addCommands(new ShoulderGoto(0.0),
        TrajectoryCommand.loadPathplannerCommand(path1, true));
  }
}
