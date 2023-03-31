// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.mayheminc.util.MayhemTalonSRX;
import org.mayheminc.util.MayhemTalonSRX.CurrentLimit;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/** Add your docs here. */
public class ClawRollers extends SubsystemBase {
    public static final double SUCK_IN = 0.25;
    private final VictorSPX talon = new VictorSPX(Constants.Talon.ROLLER_TALON);

    /** Creates a new Arm. */
    public ClawRollers() {
        configTalon(talon);
    }

    private void configTalon(VictorSPX talon) {
        talon.setNeutralMode(NeutralMode.Brake);

        // talon.configNominalOutputVoltage(+0.0f, -0.0f);
        // talon.configPeakOutputVoltage(+12.0, -12.0);

        // // configure current limits
        // // enabled = true
        // // 40 = limit (amps)
        // // 60 = trigger_threshold (amps)
        // // 0.5 = threshold time(s)
        // talon.configSupplyCurrentLimit(
        // new SupplyCurrentLimitConfiguration(
        // true,
        // 10,
        // 20,
        // 2.0));
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        // SmartDashboard.putNumber("Claw Position", get());
        // SmartDashboard.putNumber("Claw Target", talon.getClosedLoopTarget());
    }

    public double get() {
        return talon.getMotorOutputVoltage();
    }

    public void set(double p) {
        talon.set(ControlMode.PercentOutput, p);
    }
}
