package frc.robot.commands;

import java.net.http.HttpResponse.PushPromiseHandler;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class UpdateSDLimelight extends CommandBase {
    public UpdateSDLimelight() {
        addRequirements(RobotContainer.limelight);
    }

    @Override
    public void execute() {
        RobotContainer.limelight.update();

    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
