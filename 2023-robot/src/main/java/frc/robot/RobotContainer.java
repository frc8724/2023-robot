// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.AutoRoutines.*;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import frc.robot.subsystems.ArmBrake.State;

import org.mayheminc.util.LEDLights;
import org.mayheminc.util.MayhemDriverPad;
import org.mayheminc.util.MayhemOperatorPad;
import org.mayheminc.util.LEDLights.PatternID;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
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
  public static final ArmBrake armBrake = new ArmBrake();
  public static final ClawRollers clawRollers = new ClawRollers();
  public static final ClawPiston clawPiston = new ClawPiston();
  public static final Targeting targeting = new Targeting();
  public static final LimeLight limeLight = new LimeLight();
  public static final PowerDist pdp = new PowerDist();
  public static final ClawColorSensor clawColor = new ClawColorSensor();
  public static final LEDLights ledLights = new LEDLights();
  public static final Whacker whacker = new Whacker();

  MayhemDriverPad driverPad = new MayhemDriverPad();
  MayhemOperatorPad operatorPad = new MayhemOperatorPad();

  SendableChooser<Command> autoChooser = new SendableChooser<>();

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
    addAuto(new DriveSystemOnChargingStation());
    addAuto(new Week1_StandStill());

    addAuto(new Week1_DriveOut());

    addAuto(new Week1_PlaceCone_2());
    addAuto(new Week1_PlaceCone_2_DriveOut());
    addAuto(new Week1_PlaceCone_2_ChargingStation());

    addAuto(new Week1_PlaceCone_3());
    addAuto(new Week1_PlaceCone_3_DriveOut());
    addAuto(new Week1_PlaceCone_3_ChargingStation());

    SmartDashboard.putData("Auto Mode", autoChooser);
  }

  private void addAuto(Command cmd) {
    String name = cmd.getClass().getSimpleName();
    autoChooser.addOption(name, cmd);
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
        () -> driverPad.DRIVER_PAD_RIGHT_UPPER_TRIGGER_BUTTON.getAsBoolean(),
        () -> driverPad.DRIVER_PAD_RIGHT_LOWER_TRIGGER_BUTTON.getAsBoolean()));

    // driverPad.DRIVER_PAD_YELLOW_BUTTON.onTrue(new DriveBrakeMode(false));

    driverPad.DRIVER_PAD_LEFT_UPPER_TRIGGER_BUTTON.whileTrue(new DriveCenterTarget(() -> driverPad.driveThrottle()));
    driverPad.DRIVER_PAD_LEFT_UPPER_TRIGGER_BUTTON.onTrue(new LimelightSetPipeline(1));
    driverPad.DRIVER_PAD_LEFT_UPPER_TRIGGER_BUTTON.onFalse(new LimelightSetPipeline(0));

    driverPad.DRIVER_PAD_LEFT_LOWER_TRIGGER_BUTTON.onTrue(new WhackerToggle());

    driverPad.DRIVER_PAD_RIGHT_LOWER_TRIGGER_BUTTON.onTrue(new DriveBrakeMode(true));
    driverPad.DRIVER_PAD_RIGHT_LOWER_TRIGGER_BUTTON.onFalse(new DriveBrakeMode(false));

  }

  private void configureDriverStick() {
  }

  private void configureOperatorPadButtons() {
    SmartDashboard.putString("Debug", "configureOperatorPadButtons");

    operatorPad.OPERATOR_PAD_BUTTON_FOUR.whileTrue(new SystemPlaceCone(3));
    operatorPad.OPERATOR_PAD_BUTTON_THREE.whileTrue(new SystemPlaceCone(2));
    operatorPad.OPERATOR_PAD_BUTTON_TWO.whileTrue(new SystemStowArm());
    operatorPad.OPERATOR_PAD_BUTTON_ONE.whileTrue(new SystemGrabFromHumanPlayer());
    // operatorPad.OPERATOR_PAD_BUTTON_ONE.onFalse(new SystemStowArm());

    // operatorPad.OPERATOR_PAD_BUTTON_ONE.onTrue(new ShoulderGoto(68600.));
    // operatorPad.OPERATOR_PAD_BUTTON_TWO.onTrue(new ShoulderGoto(55300.));

    // operatorPad.OPERATOR_PAD_D_PAD_UP.onTrue(new LedLightsSet(PatternID.YELLOW));
    operatorPad.OPERATOR_PAD_D_PAD_UP.onTrue(new LedLightsSet(PatternID.COLOR_1_STROBE).withTimeout(10.0));
    operatorPad.OPERATOR_PAD_D_PAD_DOWN.onTrue(new LedLightsSet(PatternID.COLOR_2_STROBE).withTimeout(10.0));

    // operatorPad.OPERATOR_PAD_D_PAD_DOWN.whileTrue(new SystemGrabAndStow());
    operatorPad.OPERATOR_PAD_D_PAD_LEFT.whileTrue(new SystemFloorPickUp());

    // debug
    // operatorPad.OPERATOR_PAD_D_PAD_LEFT.onTrue(new
    // ShoulderOffsetInDegrees(-60.0)); // debug
    // operatorPad.OPERATOR_PAD_D_PAD_RIGHT.onTrue(new
    // ShoulderOffsetInDegrees(60.0)); // debug

    // Claw Rollers Left Triggers
    operatorPad.OPERATOR_PAD_BUTTON_SIX.whileTrue(new ClawRollerSet(0.5));
    operatorPad.OPERATOR_PAD_BUTTON_SIX.onFalse(new ClawRollerSet(0.1));

    // debug
    // operatorPad.OPERATOR_PAD_BUTTON_FIVE.whileTrue(new ArmBrakeSet(State.OPEN));
    // operatorPad.OPERATOR_PAD_BUTTON_FIVE.onFalse(new SequentialCommandGroup(
    // new ArmBrakeSet(State.CLOSE), new ArmSetPower(0.0)));

    operatorPad.OPERATOR_PAD_BUTTON_EIGHT.whileTrue(new ClawRollerSet(-0.5));
    operatorPad.OPERATOR_PAD_BUTTON_EIGHT.onFalse(new ClawRollerSet(0.00));

    // Claw Pistons Right Triggers
    operatorPad.OPERATOR_PAD_BUTTON_FIVE.whileTrue(new ClawPistonSet(ClawPiston.State.OPEN));
    operatorPad.OPERATOR_PAD_BUTTON_SEVEN.whileTrue(
        new SequentialCommandGroup(
            new ClawPistonSet(ClawPiston.State.CLOSE),
            new ClawRollerSet(0.1)));

    // debug
    // operatorPad.OPERATOR_PAD_BUTTON_EIGHT.onTrue(new
    // ShoulderOffsetInDegrees(-60.0)); // debug
    // operatorPad.OPERATOR_PAD_BUTTON_SIX.onTrue(new
    // ShoulderOffsetInDegrees(60.0)); // debug

    // Arm manual in/out
    operatorPad.OPERATOR_PAD_LEFT_Y_AXIS_UP.whileTrue(
        new SequentialCommandGroup(new ArmBrakeSet(State.OPEN),
            new WaitCommand(.2),
            new ArmSetPower(0.10)));
    operatorPad.OPERATOR_PAD_LEFT_Y_AXIS_UP.onFalse(new ArmBrakeSet(State.CLOSE));
    operatorPad.OPERATOR_PAD_LEFT_Y_AXIS_DOWN.whileTrue(
        new SequentialCommandGroup(new ArmBrakeSet(State.OPEN),
            new WaitCommand(.2),
            new ArmSetPower(-0.10)));
    operatorPad.OPERATOR_PAD_LEFT_Y_AXIS_DOWN.onFalse(new ArmBrakeSet(State.CLOSE));

    // Shoulder manual up/down
    operatorPad.OPERATOR_PAD_RIGHT_Y_AXIS_UP.whileTrue(new ShoulderSetPower(0.15));
    operatorPad.OPERATOR_PAD_RIGHT_Y_AXIS_DOWN.whileTrue(new ShoulderSetPower(-0.15));

    // emergency zero
    operatorPad.OPERATOR_PAD_BUTTON_TEN.onTrue(new SystemZero());

    /*
     * human player grab - cube or cone
     * set lights to yellow or purple
     * stow arm
     */
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   * Always prepend a System Zero command to the auto command.
   * 
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    RobotContainer.drive.init();
    RobotContainer.shoulder.zero();
    RobotContainer.arm.zero();

    return autoChooser.getSelected();
  }

  public Command startTeleopCommand() {
    arm.stop();
    shoulder.stop();

    return new LedLightsSet(PatternID.RED);

  }
}
