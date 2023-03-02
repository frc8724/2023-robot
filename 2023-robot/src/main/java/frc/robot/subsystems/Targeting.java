// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;

public class Targeting extends SubsystemBase {
  PIDController pid;

  final double kP = 0.01;
  final double kI = 0.0;
  final double kD = 0.0;

  double correction = 0.0;

  /** Creates a new Targeting. */
  public Targeting() {
    pid = new PIDController(kP, kI, kD);
    pid.setSetpoint(0.0);
  }

  double pidReset = 0.0;

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putBoolean("Targeting HasTarget", hasTarget());
    SmartDashboard.putNumber("Taget Horizontal", getHorizontalOffset());

    if (hasTarget()) {
      double d = -pid.calculate(getHorizontalOffset());
      d = Math.min(0.3, d);
      d = Math.max(-0.3, d);
      correction = d;

    } else {
      pid.reset();
      SmartDashboard.putNumber("Targeting Pid Reset", pidReset++);
    }

    SmartDashboard.putNumber("Targeting Correction", correction);
  }

  public boolean hasTarget() {
    return RobotContainer.limeLight.isTargetAvalible();
  }

  public double getHorizontalOffset() {
    return RobotContainer.limeLight.getX();
  }

  public double getCorrection() {
    return correction;
  }
}
