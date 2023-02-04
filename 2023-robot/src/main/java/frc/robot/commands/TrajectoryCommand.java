// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.io.IOException;
import java.nio.file.Path;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;

public class TrajectoryCommand {

  public static Command loadPathplannerCommand(String filename, boolean reseOdomtry) {
    Trajectory trajectory;

    try {
      Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(filename);
      trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
    } catch (IOException exception) {
      DriverStation.reportError("Unable to open trajectory" + filename, exception.getStackTrace());
      return new InstantCommand();
    }

    RamseteCommand ramseteCommand = new RamseteCommand(
        trajectory,
        RobotContainer.drive::getPose,
        new RamseteController(DriveTrainConstants.kRamseteB, DriveTrainConstants.kRamseteZeta),
        new SimpleMotorFeedforward(DriveTrainConstants.ksVolts, DriveTrainConstants.kvVoltSecondsPerMeter,
            DriveTrainConstants.kaVoltSecondsSquaredPerMeter),
        DriveTrainConstants.kDriveKinematics,
        RobotContainer.drive::getWheelSpeeds,
        new PIDController(DriveTrainConstants.kPDriveVel, 0, 0),
        new PIDController(DriveTrainConstants.kPDriveVel, 0, 0),
        RobotContainer.drive::tankDriveVolts,
        RobotContainer.drive);

    if (reseOdomtry) {
      return new SequentialCommandGroup(
          new InstantCommand(() -> RobotContainer.drive.resetOdometry(trajectory.getInitialPose())), ramseteCommand);
    } else {
      return ramseteCommand;
    }

  }

}
