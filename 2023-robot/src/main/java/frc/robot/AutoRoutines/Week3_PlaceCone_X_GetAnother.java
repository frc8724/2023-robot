package frc.robot.AutoRoutines;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.ArmGoto;
import frc.robot.commands.ArmSystemGoTo;
import frc.robot.commands.ArmWaitForPosition;
import frc.robot.commands.ClawDropCone;
import frc.robot.commands.ClawPistonSet;
import frc.robot.commands.ClawRollerSet;
import frc.robot.commands.DriveStraightOnHeading;
import frc.robot.commands.Drive_GoTo2ndCone;
import frc.robot.commands.Drive_Place2ndCone;
import frc.robot.commands.ShoulderGoto;
import frc.robot.commands.ShoulderWaitForPosition;
import frc.robot.commands.SystemFloorPickUp;
import frc.robot.commands.SystemPlaceCone;
import frc.robot.commands.SystemStowArm;
import frc.robot.commands.SystemZero;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Shoulder;
import frc.robot.subsystems.ClawPiston.State;

public class Week3_PlaceCone_X_GetAnother extends SequentialCommandGroup {
        public Week3_PlaceCone_X_GetAnother(int Level) {
                addCommands(new SystemZero());

                addCommands(new SystemPlaceCone(Level));

                addCommands(new ClawDropCone());

                addCommands(
                                new ParallelCommandGroup(
                                                new SystemFloorPickUp(),
                                                new Drive_GoTo2ndCone()));
                addCommands(
                                new ParallelCommandGroup(
                                                new Drive_Place2ndCone(),
                                                new SequentialCommandGroup(
                                                                new WaitCommand(1.2),
                                                                new ShoulderGoto(Shoulder.LEVEL_X_SCORE[Level]),
                                                                new ShoulderWaitForPosition(),
                                                                new ArmSystemGoTo(Arm.LEVEL_X_SCORE[Level]))));
                // new ShoulderGoto(Shoulder.LEVEL_X_SCORE[Level]))));

                addCommands(new ClawPistonSet(State.CLOSE));
                addCommands(new ClawRollerSet(-0.3));
                addCommands(new WaitCommand(.5));
                addCommands(new DriveStraightOnHeading(-.05, -.5, 20, 0));
                addCommands(new ParallelCommandGroup(
                                new DriveStraightOnHeading(-.5, 100, 0),
                                new SystemFloorPickUp()));

        }
}
