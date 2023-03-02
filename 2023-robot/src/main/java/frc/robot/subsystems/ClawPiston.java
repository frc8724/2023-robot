// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/** Add your docs here. */
public class ClawPiston extends SubsystemBase {

    public enum State {
        CLOSE,
        OPEN
    };

    private final Solenoid piston = new Solenoid(PneumaticsModuleType.CTREPCM, Constants.Solenoid.ClawPiston);

    /** Creates a new Arm. */
    public ClawPiston() {
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run

        SmartDashboard.putBoolean("Claw Piston", get() == State.OPEN);
    }

    public State get() {
        return piston.get() ? State.OPEN : State.CLOSE;
    }

    public void set(State b) {
        piston.set(b == State.OPEN);
    }

    public void zero() {
        set(State.CLOSE);
    }
}
