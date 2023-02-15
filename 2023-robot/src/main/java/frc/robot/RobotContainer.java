// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.AutoRoutines.TestTrajectoryCommand;
import frc.robot.AutoRoutines.Test_1;
import frc.robot.AutoRoutines.Test_Drive;
import frc.robot.AutoRoutines.Test_Drive_2;
import frc.robot.commands.*;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.ClawColorSensor;
import frc.robot.subsystems.ClawPiston;
import frc.robot.subsystems.ClawRollers;
import frc.robot.subsystems.DriveBaseSubsystem;
import frc.robot.subsystems.Shoulder;
import frc.robot.subsystems.Targeting;
import org.mayheminc.util.MayhemDriverPad;
import org.mayheminc.util.MayhemOperatorPad;

import frc.robot.subsystems.LimeLight;
import frc.robot.subsystems.PowerDist;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  public static final DriveBaseSubsystem drive = new DriveBaseSubsystem();
  public static final Shoulder shoulder = new Shoulder();
  public static final Arm arm = new Arm();
  public static final ClawRollers clawRollers = new ClawRollers();
  public static final ClawPiston clawPiston = new ClawPiston();
  public static final Targeting targeting = new Targeting();
  public static final LimeLight limeLight = new LimeLight();
  public static final PowerDist pdp = new PowerDist();
  public static final ClawColorSensor clawColor = new ClawColorSensor();

  MayhemDriverPad driverPad = new MayhemDriverPad();
  MayhemOperatorPad operatorPad = new MayhemOperatorPad();

  SendableChooser<Command> autoChooser = new SendableChooser<>();

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
    autoChooser.addOption("test1", new Test_1());
    autoChooser.addOption("testdrive", new Test_Drive());
    autoChooser.addOption("testdrive2", new Test_Drive_2());
    autoChooser.addOption("testtrajectorycommand", new TestTrajectoryCommand());
    SmartDashboard.putData("Auto Mode", autoChooser);
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be
   * created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
   * an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   */
  private void configureBindings() {
    configureDriverPadButtons();
    configureOperatorPadButtons();
    configureDriverStick();
  }

  private void configureDriverPadButtons() {
    drive.setDefaultCommand(new DriveDefaultCommand(
        () -> driverPad.driveThrottle(),
        () -> driverPad.steeringX(),
        () -> driverPad.DRIVER_PAD_RIGHT_UPPER_TRIGGER_BUTTON.getAsBoolean()));
    driverPad.DRIVER_PAD_GREEN_BUTTON.whileTrue(new DriveGetLevel());

    driverPad.DRIVER_PAD_RED_BUTTON.whileTrue(new DriveCenterTarget());

    /**
     * auto align (human player, cone, or cube) - left top trigger
     * auto level - left bottom trigger
     * quick turn - right top trigger
     * slow mode - right bottom trigger
     * 
     */
  }

  private void configureDriverStick() {
  }

  private void configureOperatorPadButtons() {
    operatorPad.OPERATOR_PAD_BUTTON_FOUR.whileTrue(new SystemPlaceCone(3));
    operatorPad.OPERATOR_PAD_BUTTON_THREE.whileTrue(new SystemPlaceCone(2));
    operatorPad.OPERATOR_PAD_BUTTON_TWO.whileTrue(new SystemPlaceCone(1));
    operatorPad.OPERATOR_PAD_BUTTON_ONE.whileTrue(new SystemGrabFromHumanPlayer());
    operatorPad.OPERATOR_PAD_BUTTON_ONE.onFalse(
        new ClawColorCommand(
            new ClawPistonSet(true),
            new ClawPistonSet(false)));

    // Claw Rollers Left Triggers
    operatorPad.OPERATOR_PAD_BUTTON_FIVE.whileTrue(new ClawRollerSet(0.2));
    operatorPad.OPERATOR_PAD_BUTTON_SEVEN.whileTrue(new ClawRollerSet(-0.2));

    // Claw Pistons Right Triggers
    operatorPad.OPERATOR_PAD_BUTTON_SIX.whileTrue(new ClawPistonSet(true));
    operatorPad.OPERATOR_PAD_BUTTON_EIGHT.whileTrue(new ClawPistonSet(false));

    // Arm manual up/down
    operatorPad.OPERATOR_PAD_LEFT_Y_AXIS_UP.whileTrue(new ArmSetPower(0.25));
    operatorPad.OPERATOR_PAD_LEFT_Y_AXIS_DOWN.whileTrue(new ArmSetPower(-0.25));

    // Shoulder manual up/down
    operatorPad.OPERATOR_PAD_RIGHT_Y_AXIS_UP.whileTrue(new ShoulderSetPower(0.25));
    operatorPad.OPERATOR_PAD_RIGHT_Y_AXIS_DOWN.whileTrue(new ShoulderSetPower(-0.25));

    /*
     * human player grab - cube or cone
     * set lights to yellow or purple
     * stow arm
     */
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return autoChooser.getSelected();
  }
}
