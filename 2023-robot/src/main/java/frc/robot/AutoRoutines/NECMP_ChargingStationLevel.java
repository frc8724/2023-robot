// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.AutoRoutines;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.commands.SystemZero;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class NECMP_ChargingStationLevel extends SequentialCommandGroup {
  /** Creates a new NECMP_ChargingStationLevel. */
  public NECMP_ChargingStationLevel() {
    var pitch = RobotContainer.drive.getPitch();
    // double slop = 2;
    // if (pitch>slop){
    //   RobotContainer.drive.speedRacerDrive(0.2);
    // }else if (pitch<slop){
    //   RobotContainer.drive.speedRacerDrive(-0.2);
    // }else{
    //   RobotContainer.
    // }

    

    // double START_DRIVING_PITCH = 5.0;
    // double STOP_DRIVING_PITCH = 3.0;
    // double DRIVE_POWER = 0.1;
    // double PEAK_PITCH = 0.5;
    // double lastPitch = 0.0;
    // boolean climbing = false;
    // // Add your commands in the addCommands() call, e.g.
    // // addCommands(new FooCommand(), new BarCommand());
    // addCommands(new SystemZero());
    // var pitch = RobotContainer.drive.getPitch();

    // // if we are pitched up...
    // if (pitch > 0) {
    //   if (climbing) {
    //     if (pitch < lastPitch + PEAK_PITCH) {
    //       climbing = false;
    //       RobotContainer.drive.speedRacerDrive(0, 0, true);
    //     }
    //   } else {
    //     // if it is really steep, drive forward
    //     if (pitch > START_DRIVING_PITCH) {
    //       climbing = true;
    //       RobotContainer.drive.speedRacerDrive(DRIVE_POWER, 0, true);
    //     }
    //   }
    // } else { // else we are pitched down...
    //   if (climbing) {
    //     if (pitch > lastPitch - PEAK_PITCH) {
    //       climbing = false;
    //       RobotContainer.drive.speedRacerDrive(0, 0, true);
    //     }
    //   } else {
    //     if (pitch < -START_DRIVING_PITCH) {
    //       climbing = true;
    //       RobotContainer.drive.speedRacerDrive(-DRIVE_POWER, 0, true);
    //     }
    //   }
    // }

    // lastPitch = pitch;

  }
}
