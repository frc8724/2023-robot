package frc.robot.util;

import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;

public class DisabledOnlyJoystickButton extends Button {
    private GenericHID joystick;
    private int buttonNumber;

    public DisabledOnlyJoystickButton(GenericHID joystick, int buttonNumber) {
        this.joystick = joystick;
        this.buttonNumber = buttonNumber;
    }

    @Override
    public boolean get() {
        return joystick.getRawButton(buttonNumber) && DriverStation.isDisabled();
    }
}
