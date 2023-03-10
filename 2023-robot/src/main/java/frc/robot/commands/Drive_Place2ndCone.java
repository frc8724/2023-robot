package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class Drive_Place2ndCone extends SequentialCommandGroup{
    public Drive_Place2ndCone() {
        addCommands(new DriveStraightOnHeading(0.2, 0.5, 20.0, -5.0));
        addCommands(new DriveStraightOnHeading(0.5, 120.0, 0.0));

        addCommands(new DriveStraightOnHeading(0.5, 75.0, 6.0));

        addCommands(new DriveStraightOnHeading(0.5, 0.05, 20.0, 6.0));

    }
}
