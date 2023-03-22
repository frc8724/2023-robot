// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class PowerDist extends SubsystemBase {

  PowerDistribution pdp = new PowerDistribution();

  /** Creates a new PowerDistribution. */
  public PowerDist() {
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    // RJD: commented out because it takes too long
    // updateSdbPdp();
  }

  /**
   * updateSdbPdp Update the Smart Dashboard with the Power Distribution Panel
   * currents.
   */
  void updateSdbPdp() {
    double lf;
    double rf;
    double lb;
    double rb;

    final double fudgeFactor = 0.0;
    lf = pdp.getCurrent(Constants.PDP.DRIVE_LEFT_FRONT) - fudgeFactor;
    rf = pdp.getCurrent(Constants.PDP.DRIVE_LEFT_REAR) - fudgeFactor;
    lb = pdp.getCurrent(Constants.PDP.DRIVE_RIGHT_FRONT) - fudgeFactor;
    rb = pdp.getCurrent(Constants.PDP.DRIVE_RIGHT_REAR) - fudgeFactor;

    // SmartDashboard.putNumber("Left Front I", lf);
    // SmartDashboard.putNumber("Right Front I", rf);
    // SmartDashboard.putNumber("Left Back I", lb);
    // SmartDashboard.putNumber("Right Back I", rb);
  }
}
