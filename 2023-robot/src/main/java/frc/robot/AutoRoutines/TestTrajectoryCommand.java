// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.AutoRoutines;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.DriveStraight;
import frc.robot.commands.TrajectoryCommand;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TestTrajectoryCommand extends SequentialCommandGroup {
  /** Creates a new TestTrajectoryCommand. */
  public TestTrajectoryCommand() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(TrajectoryCommand.loadPathplannerCommand(
        "pathplanner/generatedJSON/TestStraight.wpilib.json",
        true));
        addCommands(new DriveStraight(.2, 1.0));
  }
}
