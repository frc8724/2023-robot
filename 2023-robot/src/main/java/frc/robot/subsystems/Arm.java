// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.mayheminc.util.MayhemTalonFX;
import org.mayheminc.util.MayhemTalonFX.CurrentLimit;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Arm extends SubsystemBase {

  static final double TICKS_PER_INCH = 3442;
  public static final double[] LEVEL_X_SCORE = { 0.0, 2000.0, 3000.0, 3500.0 };
  public static final double HUMAN_PLAYER_STATION = 1234.0;
  public static final double STOW = 100.0;

  static final double POSITION_SLOP = 1000.0;
  static final double CLOSED_LOOP_RAMP_RATE = 1.0;

  private final MayhemTalonFX talon = new MayhemTalonFX(Constants.Talon.ARM_FALCON, CurrentLimit.HIGH_CURRENT);

  /** Creates a new Arm. */
  public Arm() {
    configTalon(talon);
    zero();
  }

  private void configTalon(TalonFX talon) {
    talon.setNeutralMode(NeutralMode.Brake);

    talon.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 0);

    talon.config_kP(0, 0.1);
    talon.config_kI(0, 0.0);
    talon.config_kD(0, 0.0);
    talon.config_kF(0, 0.0);

    talon.configClosedloopRamp(CLOSED_LOOP_RAMP_RATE); // specify minimum time for neutral to full in seconds
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

  public double getCurrentPositionInInches() {
    return getCurrentPosition() / TICKS_PER_INCH;
  }

  public double getTargetPosition() {
    return talon.getClosedLoopTarget();
  }

  public void setInInches(double p) {
    talon.set(ControlMode.Position, p * TICKS_PER_INCH);
  }

  public boolean isAtPosition() {
    return Math.abs(getCurrentPosition() - getTargetPosition()) < POSITION_SLOP;
  }

  public void stop() {
    setInInches(getCurrentPosition());
  }

  // Set the arm to horizontal and then call zero().
  public void zero() {
    DriverStation.reportWarning("Arm: zero", false);
    talon.setSelectedSensorPosition(0.0);
    talon.set(ControlMode.PercentOutput, 0.0);
  }

  public void setPower(double d) {
    talon.set(ControlMode.PercentOutput, d);
  }
}
