// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public final class Talon {
        public static final int DRIVE_LEFT_TOP = 1; // falcons FX
        public static final int DRIVE_LEFT_FRONT = 2; // falcons FX
        public static final int DRIVE_LEFT_BOTTOM = 3; // falcons FX
        public static final int DRIVE_RIGHT_TOP = 4; // falcons FX
        public static final int DRIVE_RIGHT_FRONT = 5; // falcons FX
        public static final int DRIVE_RIGHT_BOTTOM = 6; // falcons FX

        public static final int ARM_FALCON = 7; // falcons FX

        public static final int LEFT_SHOULDER_FALCON = 8; // falcons FX
        public static final int RIGHT_SHOULDER_FALCON = 9; // falcons FX

        public static final int ROLLER_TALON = 10; // Talon SRX

        public static final int INTAKE_ROLLERS_2 = 11; // Talon SRX

    }

    public final class Solenoid {
        public static final int ClawPiston = 3;
        public static final int ArmBrake = 2;
        public static final int Whacker = 1;
    }

    public final class PDP {
        public static final int DRIVE_LEFT_FRONT = 1;
        public static final int DRIVE_LEFT_REAR = 1;
        public static final int DRIVE_RIGHT_FRONT = 1;
        public static final int DRIVE_RIGHT_REAR = 1;
    }

    public final class Control {
        public static final int DRIVER_STICK_PORT = 0;
    }

    public final class DigitalInput {
        public static final int RIGHT_CLIMBER_BOTTOM_LIMIT = 1;
        public static final int RIGHT_CLIMBER_TOP_LIMIT = 0;

        public static final int LEFT_CLIMBER_TOP_LIMIT = 2;
        public static final int LEFT_CLIMBER_BOTTOM_LIMIT = 3;
    }

    public final class Lights {
        public static final int LIGHTS_PORT = 0;
    }

    public final class PdpPorts {
        public static final int DriveBaseLeft1 = 0;
        public static final int DriveBaseLeft2 = 1;
        public static final int ShooterFalconX = 2;
        public static final int ClimberLeft = 3;
        public static final int Intake = 5;
        public static final int UnusedTalon = 6;
        public static final int Maggie = 7;
        public static final int Loader = 9;
        public static final int Shooter775X = 10;
        public static final int Shooter775Y = 11;
        public static final int ClimberR = 12;
        public static final int ShooterRight = 13;
        public static final int DriveBaseRight1 = 14;
        public static final int DriveBaseRight2 = 15;
    }

    public static double ConvertFeetToMeters(double ft) {
        return ft * 12 / 0.0254; // convert to inches, convert inches to meters
    }
}
