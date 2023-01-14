package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class pipelinechange extends InstantCommand {

    public pipelinechange() {
        addRequirements(RobotContainer.limelight);
    }

    @Override
    public void execute() {
        RobotContainer.limelight.pipelineMode(Constants.currentPipeline);
        int i = Constants.currentPipeline;
        i = i + 1;
        if (i == 3) {
            i = 0;
        }
        Constants.currentPipeline = i;
    }

}
