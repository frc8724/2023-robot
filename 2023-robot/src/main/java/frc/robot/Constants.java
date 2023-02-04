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
        public static final int DRIVE_LEFT_1 = 1; // falcons FX
        public static final int DRIVE_LEFT_2 = 2; // falcons FX
        public static final int DRIVE_LEFT_3 = 3; // falcons FX
        public static final int DRIVE_RIGHT_1 = 4; // falcons FX
        public static final int DRIVE_RIGHT_2 = 5; // falcons FX
        public static final int DRIVE_RIGHT_3 = 6; // falcons FX

        public static final int ARM_FALCON = 7; // falcons FX

        public static final int LEFT_SHOULDER_FALCON = 8; // falcons FX
        public static final int RIGHT_SHOULDER_FALCON = 9; // falcons FX


    }

    public final class Solenoid {
        public static final int ClawPiston = 0;
      
    }

    public final class PDP {
        public static final int DRIVE_LEFT_1 = 1;
        public static final int DRIVE_LEFT_2 = 2;
        public static final int DRIVE_RIGHT_1 = 15;
        public static final int DRIVE_RIGHT_2 = 14;
    }

    public final class Control {
        public static final int DRIVER_STICK_PORT = 0;
    }

    public final class DigitalInput {
       
    }

    public final class Lights {
        public static final int LIGHTS_PORT = 0;
    }

}
