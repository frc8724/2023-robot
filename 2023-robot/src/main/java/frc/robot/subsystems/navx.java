package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.*;

public class navx extends SubsystemBase {
    private AHRS navx;

    public navx() {
        try {
            /* Communicate w/navX MXP via the MXP SPI Bus. */
            /* Alternatively: I2C.Port.kMXP, SerialPort.Port.kMXP or SerialPort.Port.kUSB */
            /*
             * See http://navx-mxp.kauailabs.com/guidance/selecting-an-interface/ for
             * details.
             */
            navx = new AHRS(SPI.Port.kMXP);
            navx.reset();
        } catch (final RuntimeException ex) {
            DriverStation.reportError("Error instantiating navX MXP:  " + ex.getMessage(), true);
            System.out.println("Error loading navx.");
        }
    }

    public double getYaw() {
        return navx.getYaw();
    }

    public double getRoll() {
        return navx.getRoll();
    }

    public double getPitch() {
        return navx.getPitch();
    }

    public void updateSmartDashboard() {
        SmartDashboard.putNumber("Yaw", getYaw());
        SmartDashboard.putNumber("Roll", getRoll());
        SmartDashboard.putNumber("Pitch", getPitch());
    }
}
