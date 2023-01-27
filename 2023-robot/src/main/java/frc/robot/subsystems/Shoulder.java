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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shoulder extends SubsystemBase {
  final double kWheelP = 0.020;
  final double kWheelI = 0.000;
  final double kWheelD = 0.200;
  final double kWheelF = 0.060;

  double wheelP = kWheelP;
  double wheelI = kWheelP;
  double wheelD = kWheelP;
  double wheelF = kWheelP;

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

  // Ideas to tune the arm.
  // 0. Zero the Arm so that 0 degrees is horizontal.
  // 1. Rotate the Arm to horizontal (0 degrees). Note the talon ticks.
  // 2. Rotate the Arm to vertical (90 degrees). Calculate the TICKS_PER_DEGREE.
  // 3. Command the Arm to go to 0.
  // 4. Update wheelF until it is stable at 0 degrees.

  private void configureDriveTalon(final MayhemTalonSRX talon) {

    final int slot = 0;
    final int timeout = 0;
    final int TICKS_PER_DEGREE = 100; // TODO: set this to the correct value.

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
    SmartDashboard.putNumber("Arm Ticks", rightTalon.getSelectedSensorPosition());
    SmartDashboard.putNumber("Arm Target Position", rightTalon.getClosedLoopTarget());

    wheelP = SmartDashboard.getNumber("Arm P", kWheelP);
    SmartDashboard.putNumber("Arm P", wheelP);
    wheelI = SmartDashboard.getNumber("Arm I", kWheelI);
    SmartDashboard.putNumber("Arm I", wheelI);
    wheelD = SmartDashboard.getNumber("Arm D", kWheelD);
    SmartDashboard.putNumber("Arm D", wheelD);
    wheelF = SmartDashboard.getNumber("Arm F", kWheelF);
    SmartDashboard.putNumber("Arm F", wheelF);

  }

  public void set(double degree) {
  }

  public double get() {
    return 0.0;
  }

  // Set the arm to horizontal and then call zero().
  public void zero() {
    DriverStation.reportWarning("Arm: zero", false);
    rightTalon.setPosition(0);
  }
}
