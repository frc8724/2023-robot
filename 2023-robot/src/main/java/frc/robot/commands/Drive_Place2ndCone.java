package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class Drive_Place2ndCone extends SequentialCommandGroup {
    public Drive_Place2ndCone(int color) {
        addCommands(new DriveStraightOnHeading(0.2, 20.0, -45.0 * color));

        addCommands(new DriveStraightOnHeading(0.2, 0.5, 30.0, 0.0 * color));
        addCommands(new DriveStraightOnHeading(0.5, 100.0, 0.0 * color));

        addCommands(new DriveStraightOnHeading(0.5, 60.0, 0.0 * color));

        addCommands(new DriveStraightOnHeading(0.5, 0.05, 25.0, 0.0 * color));

    }
}
