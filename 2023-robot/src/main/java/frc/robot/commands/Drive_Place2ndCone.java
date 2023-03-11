package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class Drive_Place2ndCone extends SequentialCommandGroup {
    public Drive_Place2ndCone(int color) {
        addCommands(new DriveStraightOnHeading(0.2, 0.5, 20.0, -5.0 * color));
        addCommands(new DriveStraightOnHeading(0.5, 100.0, -5.0 * color));

        addCommands(new DriveStraightOnHeading(0.5, 90.0, 0.0 * color));

        addCommands(new DriveStraightOnHeading(0.5, 0.05, 25.0, 0.0 * color));

    }
}
