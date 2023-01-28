// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;

public class Targeting extends SubsystemBase {
  // NetworkTable limelight;
  PIDController pid;

  final double kP = 0.03;
  final double kI = 0.0;
  final double kD = 0.001;

  double targetP;
  double targetI;
  double targetD;
  double correction = 0.0;

  /** Creates a new Targeting. */
  public Targeting() {
    // limelight = NetworkTableInstance.getDefault().getTable("limelight");
    pid = new PIDController(kP, kI, kD);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putBoolean("Targeting HasTarget", hasTarget());
    SmartDashboard.putNumber("Taget Horizontal", getHorizontalOffset());

    if (hasTarget()) {
      correction = -pid.calculate(getHorizontalOffset());
    } else {
      pid.reset();
    }

    targetP = SmartDashboard.getNumber("Target P", kP);
    SmartDashboard.putNumber("Target P", targetP);
    targetI = SmartDashboard.getNumber("Target I", kI);
    SmartDashboard.putNumber("Target I", targetI);
    targetD = SmartDashboard.getNumber("Target D", kD);
    SmartDashboard.putNumber("Target D", targetD);

    if (pid.getP() != targetP ||
        pid.getI() != targetI ||
        pid.getD() != targetD) {
      pid = new PIDController(targetP, targetI, targetD);
    }

    SmartDashboard.putNumber("Targeting Correction", correction);

  }

  public boolean hasTarget() {
    // double d = limelight.getEntry("tv").getDouble(0.0);
    return RobotContainer.limeLight.isTargetAvalible();
    // return d > 0.5; // convert 0|1 to false|true
  }

  double getHorizontalOffset() {
    return RobotContainer.limeLight.getX();
  }

  public double getCorrection() {
    return correction;
  }
}
