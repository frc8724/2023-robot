// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

// import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.*;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.ClawPiston;
import frc.robot.subsystems.ClawRollers;
import frc.robot.subsystems.DriveBaseSubsystem;
import frc.robot.subsystems.Shoulder;

import org.mayheminc.util.MayhemDriverPad;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
//import frc.robot.commands.DrivebaseTeleop;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.LevelChargingStation;
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
  MayhemDriverPad driverPad = new MayhemDriverPad();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  // private final CommandXboxController m_driverController =
  // new CommandXboxController(OperatorConstants.kDriverControllerPort);
  public static final navx navx = new navx();
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();

  // public static final limelight limelight = new limelight();
  private final ExampleCommand m_autoCommand = new ExampleCommand(m_exampleSubsystem);

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
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    // new Trigger(m_exampleSubsystem::exampleCondition)
    // .onTrue(new ExampleCommand(m_exampleSubsystem));

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is
    // pressed,
    // cancelling on release.
    // m_driverController.b().whileTrue(m_exampleSubsystem.exampleMethodCommand());

    configureDriverPad();
  }

  private void configureDriverPad() {
    drive.setDefaultCommand(new DriveDefaultCommand(
        () -> driverPad.driveThrottle(),
        () -> driverPad.steeringX(),
        () -> driverPad.DRIVER_PAD_RIGHT_UPPER_TRIGGER_BUTTON.getAsBoolean()));
    driverPad.DRIVER_PAD_GREEN_BUTTON.whileTrue(new DriveGetLevel());
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
