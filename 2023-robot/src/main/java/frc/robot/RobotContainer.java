// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.commands.ExampleCommand;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.limelight;
import frc.robot.util.DriverPad;
import frc.robot.util.DriverStick;
import frc.robot.util.OperatorPad;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.UpdateSDLimelight;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  public static final DriverStick DRIVER_STICK = new DriverStick();
  public static final DriverPad DRIVER_PAD = new DriverPad();
  public static final OperatorPad OPERATOR_PAD = new OperatorPad();
  @SuppressWarnings("PMD.UnusedPrivateField") // TODO dont know if i need this

  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();

  public static final limelight limelight = new limelight();
  private final ExampleCommand m_autoCommand = new ExampleCommand(m_exampleSubsystem);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
    limelight.setDefaultCommand(new UpdateSDLimelight());
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
   * it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    configureDriverPadButtons();
    configureOperatorPadButtons();
    configureDriverStick();

  }

  private void configureDriverStick() {

  }

  private void configureOperatorPadButtons() {

  }

  private void configureDriverPadButtons() {

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return m_autoCommand;
  }
}
