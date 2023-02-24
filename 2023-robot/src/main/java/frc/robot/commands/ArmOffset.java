// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.RobotContainer;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ArmOffset extends InstantCommand {
  double inches;

  public ArmOffset(double inches) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.arm);

    this.inches = inches;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    double pos;
    pos = RobotContainer.arm.getCurrentPositionInTicks();
    pos += this.inches;
    RobotContainer.arm.setInTicks (pos);
    SmartDashboard.putString("Debug", "New Arm Pos: " + pos);
  }
}
