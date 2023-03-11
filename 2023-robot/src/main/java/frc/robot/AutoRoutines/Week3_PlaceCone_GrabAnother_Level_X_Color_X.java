// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.AutoRoutines;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.ArmSystemGoTo;
import frc.robot.commands.ClawDropCone;
import frc.robot.commands.Drive_GoTo2ndCone;
import frc.robot.commands.SystemFloorPickUp;
import frc.robot.commands.SystemPlaceCone;
import frc.robot.subsystems.Arm;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Week3_PlaceCone_GrabAnother_Level_X_Color_X extends SequentialCommandGroup {
  /** Creates a new Week3_PlaceCone_GrabAnother_Level_X_Color_X. */
  public Week3_PlaceCone_GrabAnother_Level_X_Color_X(int Level, int color) {
    addCommands(new SystemPlaceCone(Level));

    addCommands(new ClawDropCone());

    addCommands(
        new ParallelCommandGroup(
            new SequentialCommandGroup(
                new ArmSystemGoTo(Arm.ALMOST_STOW),
                new SystemFloorPickUp()),
            new Drive_GoTo2ndCone(color)));
  }
}
