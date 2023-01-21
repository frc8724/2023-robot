// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.mayheminc.util.MayhemTalonSRX;
import org.mayheminc.util.MayhemTalonSRX.CurrentLimit;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shoulder extends SubsystemBase {
  private final MayhemTalonSRX leftTalon = new MayhemTalonSRX(Constants.Talon.LEFT_SHOULDER_FALCON,
      CurrentLimit.HIGH_CURRENT);
  private final MayhemTalonSRX rightTalon = new MayhemTalonSRX(Constants.Talon.RIGHT_SHOULDER_FALCON,
      CurrentLimit.HIGH_CURRENT);
  private static final double CLOSED_LOOP_RAMP_RATE = 0.1; // time from neutral to full in seconds

  /** Creates a new Shoulder. */
  public Shoulder() {
    configTalon(leftTalon);
    configTalon(rightTalon);

    leftTalon.follow(rightTalon);
    leftTalon.setInverted(true);
    rightTalon.setInverted(false);
    configureDriveTalon(rightTalon);
    configureDriveTalon(leftTalon);
  }

  private void configTalon(MayhemTalonSRX talon) {
    talon.setNeutralMode(NeutralMode.Coast);

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

  private void configureDriveTalon(final MayhemTalonSRX talon) {
    final double wheelP = 0.020;
    final double wheelI = 0.000;
    final double wheelD = 0.200;
    final double wheelF = 0.060;
    final int slot = 0;
    final int timeout = 0;

    talon.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 0);

    talon.configNominalOutputVoltage(+0.0f, -0.0f);
    talon.configPeakOutputVoltage(12, -12);

    talon.config_kP(slot, wheelP, timeout);
    talon.config_kI(slot, wheelI, timeout);
    talon.config_kD(slot, wheelD, timeout);
    talon.config_kF(slot, wheelF, timeout);
    talon.configClosedloopRamp(CLOSED_LOOP_RAMP_RATE); // specify minimum time for neutral to full in seconds

    DriverStation.reportError("setWheelPIDF: " + wheelP + " " + wheelI + " " + wheelD + " " + wheelF + "\n", false);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void set(double degree) {
  }

  public double get() {
    return 0.0;
  }
}
