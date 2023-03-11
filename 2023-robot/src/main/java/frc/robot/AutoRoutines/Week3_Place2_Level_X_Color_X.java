package frc.robot.AutoRoutines;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.Color;
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

public class Week3_Place2_Level_X_Color_X extends SequentialCommandGroup {
        public Week3_Place2_Level_X_Color_X(int Level, int color) {
                addCommands(new SystemZero());

                addCommands(new Week3_PlaceCone_GrabAnother_Level_X_Color_X(Level,color));

                addCommands(
                                new ParallelCommandGroup(
                                                new Drive_Place2ndCone(color),
                                                new SequentialCommandGroup(
                                                                new WaitCommand(1.2),
                                                                new ShoulderGoto(Shoulder.LEVEL_X_PRESCORE[Level]),
                                                                new ShoulderWaitForPosition(),
                                                                new ArmSystemGoTo(Arm.LEVEL_X_SCORE[Level]))));
                // new ShoulderGoto(Shoulder.LEVEL_X_SCORE[Level]))));

                addCommands(new ClawPistonSet(State.CLOSE));
                addCommands(new ClawRollerSet(-0.3));
                addCommands(new WaitCommand(.5));
                addCommands(new DriveStraightOnHeading(-.05, -.5, 20, 0 * color));
                addCommands(new ParallelCommandGroup(
                                new DriveStraightOnHeading(-.5, 100, 0 * color),
                                new SequentialCommandGroup(
                                                new ArmSystemGoTo(Arm.ALMOST_STOW),
                                                new SystemFloorPickUp())));

        }
}
