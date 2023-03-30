// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.AutoRoutines;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.commands.SystemZero;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class NECMP_ChargingStationLevel extends CommandBase {
  /** Creates a new NECMP_ChargingStationLevel. */
  boolean isFinished = false;
  public NECMP_ChargingStationLevel() {
    var pitch = RobotContainer.drive.getPitch();
    double slop = 2;
    RobotContainer.drive.setBrake(true);
    if (pitch>slop){
      RobotContainer.drive.speedRacerDrive(0.2);
    }else if (pitch<-slop){
      RobotContainer.drive.speedRacerDrive(-0.2);
    }else{
      RobotContainer.drive.speedRacerDrive(0.0);
     
      isFinished = true;
    }

  }
  @Override
  public boolean isFinished() {
    return isFinished;
  }
}
