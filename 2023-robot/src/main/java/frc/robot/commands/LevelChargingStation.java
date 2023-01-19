package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class LevelChargingStation extends CommandBase {
    public LevelChargingStation() {
        addRequirements(RobotContainer.navx);
    }

    @Override
    public void execute() {
        double x = RobotContainer.navx.getPitch();
        if (x > 100 || x < -100) {
            new DriveStraight(0.2);
        }

    }

}
