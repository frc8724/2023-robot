// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/** Add your docs here. */
public class ArmBrake extends SubsystemBase {

    public enum State {
        CLOSE,
        OPEN
    };

    public static final double OPEN_TIME_SEC = 0.1;

    // private final Solenoid brakeClose = new
    // Solenoid(PneumaticsModuleType.CTREPCM, Constants.Solenoid.ArmBrakeClose);
    private final Solenoid brake = new Solenoid(PneumaticsModuleType.CTREPCM, Constants.Solenoid.ArmBrake);

    /** Creates a new Arm. */
    public ArmBrake() {
        zero();
    }

    public void zero() {
        set(State.CLOSE);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run

        // SmartDashboard.putBoolean("Arm Brake Piston", get() == State.OPEN);
    }

    public State get() {
        return brake.get() ? State.OPEN : State.CLOSE;
    }

    public void set(State b) {
        brake.set(b == State.OPEN);
    }
}
