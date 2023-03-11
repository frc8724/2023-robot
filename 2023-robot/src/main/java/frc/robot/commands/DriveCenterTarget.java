// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.RobotContainer;

public class DriveCenterTarget extends CommandBase {
  Supplier<Double> throttle;
  PIDController m_HeadingPid;

  private static final double HEADING_PID_P = 0.050; // was 0.007 at GSD; was 0.030 in 2019 for HIGH_GEAR
  private static final double HEADING_PID_I = 0.000; // was 0.000 at GSD; was 0.000 in 2019
  private static final double HEADING_PID_D = 0.000; // was 0.080 at GSD; was 0.04 in 2019

  /** Creates a new DriveCenterTarget. */
  public DriveCenterTarget(Supplier<Double> throttle) {
    addRequirements(RobotContainer.drive);
    addRequirements(RobotContainer.targeting);
    addRequirements(RobotContainer.limeLight);
    this.throttle = throttle;

    m_HeadingPid = new PIDController(HEADING_PID_P, HEADING_PID_I, HEADING_PID_D, 0.020 /* period in seconds */);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    RobotContainer.limeLight.pipelineMode(2);
    // RobotContainer.limeLight.ledMode(true);
    m_HeadingPid.reset();

    // double desiredHeading = RobotContainer.targeting.getHorizontalOffset();
    m_HeadingPid.setSetpoint(0.0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double t = throttle.get();
    if (RobotContainer.targeting.hasTarget()) {
      // double currentHeading = RobotContainer.drive.getHeading();
      double desiredHeading = RobotContainer.targeting.getHorizontalOffset();
      // m_HeadingPid.setSetpoint(desiredHeading);
      double steering;// = error * .04;

      steering = m_HeadingPid.calculate(desiredHeading);

      // SmartDashboard.putNumber("Drive Center Error", desiredHeading -
      // currentHeading);
      SmartDashboard.putNumber("Drive Center steering", steering);

      SmartDashboard.putNumber("Drive Center H Offset", desiredHeading);
      // SmartDashboard.putNumber("Drive Center Robot Heading", currentHeading);

      RobotContainer.drive.speedRacerDrive(t, -steering, true);

    } else {
      RobotContainer.drive.speedRacerDrive(t, 0.0, false);
      m_HeadingPid.reset();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.drive.speedRacerDrive(0, 0, false);
    // RobotContainer.limeLight.ledMode(false);
    RobotContainer.limeLight.pipelineMode(3);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  // double getRotationSpeed() {
  // double h = RobotContainer.targeting.getCorrection();
  // return h;
  // }
}
