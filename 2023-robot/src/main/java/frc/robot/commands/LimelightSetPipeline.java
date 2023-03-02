// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.RobotContainer;

public class LimelightSetPipeline extends InstantCommand {
  int pipeline;

  public LimelightSetPipeline(int pipeline) {
    addRequirements(RobotContainer.limeLight);
    this.pipeline = pipeline;
  }

  @Override
  public void initialize() {
    RobotContainer.limeLight.pipelineMode(pipeline);
  }
}
