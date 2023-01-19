package org.mayheminc.util;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.button.*;

/**
 *
 * @author Team1519
 */
public class DisabledOnlyJoystickButton extends Trigger {
    public DisabledOnlyJoystickButton(GenericHID joystick, int buttonNumber) {
        super(() -> joystick.getRawButton(buttonNumber) && DriverStation.isDisabled());
    }
}
