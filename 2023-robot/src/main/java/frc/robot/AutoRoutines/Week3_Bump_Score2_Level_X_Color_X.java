// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.AutoRoutines;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.ArmSystemGoTo;
import frc.robot.commands.ClawDropCone;
import frc.robot.commands.ClawPistonSet;
import frc.robot.commands.ClawRollerSet;
import frc.robot.commands.SystemFloorPickUp;
import frc.robot.commands.SystemPlaceCone;
import frc.robot.commands.SystemZero;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.ClawPiston.State;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Week3_Bump_Score2_Level_X_Color_X extends SequentialCommandGroup {
  /** Creates a new Week3_Bump_Score2_Level_X_Color_X. */
  public Week3_Bump_Score2_Level_X_Color_X(int Level, int color) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new SystemZero());
    addCommands(new SystemPlaceCone(Level));

    addCommands(new ClawDropCone());

    addCommands(
        new ParallelCommandGroup(
            new SequentialCommandGroup(
                new ArmSystemGoTo(Arm.ALMOST_STOW),
                new SystemFloorPickUp()),
            new Drive_GoToBump2ndCube(color)));
    addCommands(new Drive_Place2ndCubeBump(Level, color));
  }
}
