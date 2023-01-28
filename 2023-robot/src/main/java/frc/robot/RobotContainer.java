// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

//import frc.robot.commands.DrivebaseTeleop;
// import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.*;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.ClawPiston;
import frc.robot.subsystems.ClawRollers;
import frc.robot.subsystems.DriveBaseSubsystem;
import frc.robot.subsystems.Shoulder;
import frc.robot.subsystems.Targeting;

import org.mayheminc.util.MayhemDriverPad;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
//import frc.robot.subsystems.Drivebase;
import frc.robot.subsystems.ExampleSubsystem;
// import frc.robot.subsystems.limelight;
import frc.robot.subsystems.navx;
// import frc.robot.util.DriverPad;
// import frc.robot.util.DriverStick;
// import frc.robot.util.OperatorPad;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
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
  // private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  public static final DriveBaseSubsystem drive = new DriveBaseSubsystem();
  public static final Shoulder shoulder = new Shoulder();
  public static final Arm arm = new Arm();
  public static final ClawRollers clawRollers = new ClawRollers();
  public static final ClawPiston clawPiston = new ClawPiston();
  public static final Targeting targeting = new Targeting();

  MayhemDriverPad driverPad = new MayhemDriverPad();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  // private final CommandXboxController m_driverController =
  // new CommandXboxController(OperatorConstants.kDriverControllerPort);
  public static final navx navx = new navx();

  // public static final limelight limelight = new limelight();

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();

  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be
   * created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
   * an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link
   * CommandXboxController
   * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or
   * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {

    configureDriverPad();
    configureOperatorPadButtons();
    configureDriverStick();
  }

  private void configureDriverPad() {
    drive.setDefaultCommand(new DriveDefaultCommand(
        () -> driverPad.driveThrottle(),
        () -> driverPad.steeringX(),
        () -> driverPad.DRIVER_PAD_RIGHT_UPPER_TRIGGER_BUTTON.getAsBoolean()));
    driverPad.DRIVER_PAD_GREEN_BUTTON.whileTrue(new DriveGetLevel());

    driverPad.DRIVER_PAD_RED_BUTTON.whileTrue(new DriveCenterTarget());

    // Test Buttons.
    driverPad.DRIVER_PAD_YELLOW_BUTTON.whileTrue(new DriveRotate(0.2));
    driverPad.DRIVER_PAD_BLUE_BUTTON.whileTrue(new DriveRotate(-0.2));

  }

  private void configureDriverStick() {

  }

  private void configureOperatorPadButtons() {
    // OPERATOR_PAD.OPERATOR_PAD_BUTTON_ONE.whenPressed(new pipelinechange());
  }

  private void configureDriverPadButtons() {
    // DRIVER_PAD.DRIVER_PAD_YELLOW_BUTTON.whileHeld(new LevelChargingStation());

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return Autos.exampleAuto(drive);
  }
}
