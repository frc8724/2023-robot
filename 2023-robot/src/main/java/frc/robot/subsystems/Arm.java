// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.mayheminc.util.MayhemTalonSRX;
import org.mayheminc.util.MayhemTalonSRX.CurrentLimit;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Arm extends SubsystemBase {

  public static final double[] LEVEL_X_SCORE = { 0.0, 2000.0, 3000.0, 3500.0 };
  public static final double HUMAN_PLAYER_STATION = 1234.0;
  public static final double STOW = 100.0;

  static final double POSITION_SLOP = 1000.0;

  private final MayhemTalonSRX talon = new MayhemTalonSRX(Constants.Talon.ARM_FALCON,
      CurrentLimit.HIGH_CURRENT);

  /** Creates a new Arm. */
  public Arm() {
    configTalon(talon);
  }

  private void configTalon(MayhemTalonSRX talon) {
    talon.setNeutralMode(NeutralMode.Brake);

    talon.configPeakCurrentLimit(60);
    talon.configContinuousCurrentLimit(40);
    talon.configPeakCurrentDuration(3000);

    talon.configNominalOutputVoltage(+0.0f, -0.0f);
    talon.configPeakOutputVoltage(+12.0, -12.0);

    // configure current limits
    // enabled = true
    // 40 = limit (amps)
    // 60 = trigger_threshold (amps)
    // 0.5 = threshold time(s)
    talon.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 40, 60, 0.5));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Arm Position", getCurrentPosition());
    SmartDashboard.putNumber("Arm Target", getTargetPosition());
  }

  public double getCurrentPosition() {
    return talon.getSelectedSensorPosition();
  }

  public double getTargetPosition() {
    return talon.getClosedLoopTarget();
  }

  public void set(double p) {
    talon.set(ControlMode.Position, p);
  }

  public boolean isAtPosition() {
    return Math.abs(getCurrentPosition() - getTargetPosition()) < POSITION_SLOP;
  }

  public void stop() {
    set(getCurrentPosition());
  }

  // Set the arm to horizontal and then call zero().
  public void zero() {
    DriverStation.reportWarning("Arm: zero", false);
    talon.setPosition(0);
  }

  public void setPower(double d) {
    talon.set(ControlMode.PercentOutput, d);
  }
}
