package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class Drive_GoTo2ndCone extends SequentialCommandGroup {
    public Drive_GoTo2ndCone(int color) {
        // drive to center line
        addCommands(new DriveStraightOnHeading(-0.05, -0.5, 20.0, 0.0 * color));
        addCommands(new DriveStraightOnHeading(-0.5, 160.0, 2.0 * color));
        addCommands(new DriveStraightOnHeading(-0.5, -0.2, 20.0, 2.0 * color));

        // turn and get cone/cube
        addCommands(new DriveStraightOnHeading(-0.2, 25.0, 50.0 * color));

        addCommands(new DriveStraightOnHeading(0.05, 0.2, 5.0, 85.0 * color));
        addCommands(new DriveStraightOnHeading(0.2, 15.0, 85.0 * color));
    }

}
