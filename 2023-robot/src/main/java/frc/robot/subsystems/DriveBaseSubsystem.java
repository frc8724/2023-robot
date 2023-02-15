package frc.robot.subsystems;

import frc.robot.Constants;
import org.mayheminc.util.History;
import org.mayheminc.util.MayhemTalonSRX;
import org.mayheminc.util.MayhemTalonSRX.CurrentLimit;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.*;
import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveBaseSubsystem extends SubsystemBase {
    History headingHistory = new History();

    // Brake modes
    public static final boolean BRAKE_MODE = true;
    public static final boolean COAST_MODE = false;

    HeadingCorrection headingCorrection = new HeadingCorrection();

    // Talons
    private final MayhemTalonSRX leftTalon1 = new MayhemTalonSRX(Constants.Talon.DRIVE_LEFT_TOP,
            CurrentLimit.HIGH_CURRENT);
    private final MayhemTalonSRX leftTalon2 = new MayhemTalonSRX(Constants.Talon.DRIVE_LEFT_FRONT,
            CurrentLimit.HIGH_CURRENT);
    private final MayhemTalonSRX leftTalon3 = new MayhemTalonSRX(Constants.Talon.DRIVE_LEFT_BOTTOM,
            CurrentLimit.HIGH_CURRENT);
    private final MayhemTalonSRX rightTalon1 = new MayhemTalonSRX(Constants.Talon.DRIVE_RIGHT_TOP,
            CurrentLimit.HIGH_CURRENT);
    private final MayhemTalonSRX rightTalon2 = new MayhemTalonSRX(Constants.Talon.DRIVE_RIGHT_FRONT,
            CurrentLimit.HIGH_CURRENT);
    private final MayhemTalonSRX rightTalon3 = new MayhemTalonSRX(Constants.Talon.DRIVE_RIGHT_BOTTOM,
            CurrentLimit.HIGH_CURRENT);

    // Odometry class for tracking robot pose
    private final DifferentialDriveOdometry m_odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(0.0), 0.0,
            0.0);

    // Drive parameters
    // pi * diameter * (pulley ratios) / (counts per rev * gearbox reduction)
    public static final double INCHES_TO_METER_CONVERSION_FACTOR = 0.0254;
    public static final double DISTANCE_PER_PULSE_IN_INCHES = 3.14 * 6.0 * 36.0 / 42.0 / (2048.0 * 7.56); // corrected
    public static final double DISTANCE_PER_PULSE_IN_METERS = DISTANCE_PER_PULSE_IN_INCHES
            * INCHES_TO_METER_CONVERSION_FACTOR;
    public static final double DISTANCE_PER_ROTATION_IN_METERS = DISTANCE_PER_PULSE_IN_METERS * 2048;

    private boolean m_closedLoopMode = true;
    private final double m_maxWheelSpeed = 18000.0; // should be maximum wheel speed in native units
    private static final double CLOSED_LOOP_RAMP_RATE = 1.0; // time from neutral to full in seconds
    private static final double OPEN_LOOP_RAMP_RATE = 1.0; // time from neutral to full in seconds

    private double m_initialWheelDistance = 0.0;
    private int m_iterationsSinceRotationCommanded = 0;
    private int m_iterationsSinceMovementCommanded = 0;

    private static final int LOOPS_GYRO_DELAY = 10;

    double m_lastLeftPercent;
    double m_lastRightPercent;

    /***********************************
     * INITIALIZATION
     **********************************************************/

    public DriveBaseSubsystem() {
        // confirm all four drive talons are in coast mode

        this.configTalon(leftTalon1);
        this.configTalon(leftTalon2);
        this.configTalon(leftTalon3);
        this.configTalon(rightTalon1);
        this.configTalon(rightTalon2);
        this.configTalon(rightTalon3);

        // set rear talons to follow their respective front talons
        leftTalon2.follow(leftTalon1);
        leftTalon3.follow(leftTalon1);
        rightTalon2.follow(rightTalon1);
        rightTalon3.follow(rightTalon1);

        // the left motors move the robot forwards with positive power
        // but the right motors are backwards.
        leftTalon1.setInverted(false);
        leftTalon2.setInverted(false);
        leftTalon3.setInverted(false);
        rightTalon1.setInverted(true);
        rightTalon2.setInverted(true);
        rightTalon3.setInverted(true);

        // talon closed loop config
        configureDriveTalon(leftTalon1);
        configureDriveTalon(rightTalon1);

        headingCorrection.zeroHeadingGyro(0.0);
    }

    private void configTalon(MayhemTalonSRX talon) {
        talon.setNeutralMode(NeutralMode.Coast);

        talon.configNominalOutputVoltage(+0.0f, -0.0f);
        talon.configPeakOutputVoltage(+12.0, -12.0);

        // configure current limits
        // enabled = true
        // 40 = limit (amps)
        // 60 = trigger_threshold (amps)
        // 0.5 = threshold time(s)
        talon.configSupplyCurrentLimit(
                new SupplyCurrentLimitConfiguration(
                        true,
                        40,
                        60,
                        1.0));
    }

    public void init() {
        // reset the NavX
        headingCorrection.zeroHeadingGyro(0.0);
    }

    private void configureDriveTalon(final MayhemTalonSRX talon) {
        final double wheelP = 0.020;
        final double wheelI = 0.000;
        final double wheelD = 0.200;
        final double wheelF = 0.060;
        final int slot = 0;
        final int timeout = 0;

        talon.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 0);

        talon.configNominalOutputVoltage(+0.0f, -0.0f);
        talon.configPeakOutputVoltage(12, -12);

        talon.config_kP(slot, wheelP, timeout);
        talon.config_kI(slot, wheelI, timeout);
        talon.config_kD(slot, wheelD, timeout);
        talon.config_kF(slot, wheelF, timeout);

        talon.configClosedloopRamp(CLOSED_LOOP_RAMP_RATE); // specify minimum time for neutral to full in seconds
        talon.configOpenloopRamp(OPEN_LOOP_RAMP_RATE); // set a reasonable open loop ramp

        DriverStation.reportError("setWheelPIDF: " + wheelP + " " + wheelI + " " + wheelD + " " + wheelF + "\n", false);
    }

    // *********************** CLOSED-LOOP MODE ********************************

    public void toggleClosedLoopMode() {
        if (!m_closedLoopMode) {
            setClosedLoopMode();
        } else {
            setOpenLoopMode();
        }
    }

    public void setClosedLoopMode() {
        m_closedLoopMode = true;
    }

    public void setOpenLoopMode() {
        m_closedLoopMode = false;
    }

    // ********************* ENCODER-GETTERS ************************************

    private double getRightEncoder() {
        return rightTalon1.getSelectedSensorPosition(0);
    }

    private double getLeftEncoder() {
        return leftTalon1.getSelectedSensorPosition(0);
    }

    static private final double STATIONARY = 0.1;
    static private double m_prevLeftDistance = 0.0;
    static private double m_prevRightDistance = 0.0;

    public boolean isStationary() {
        final double leftDistance = getLeftEncoder();
        final double rightDistance = getRightEncoder();

        final double leftDelta = Math.abs(leftDistance - m_prevLeftDistance);
        final double rightDelta = Math.abs(rightDistance - m_prevRightDistance);

        m_prevLeftDistance = leftDistance;
        m_prevRightDistance = rightDistance;

        return leftDelta < STATIONARY && rightDelta < STATIONARY;
    }

    public void stop() {
        setMotorPower(0.0, 0.0);
    }

    private void setMotorPower(double leftPower, double rightPower) {
        if (rightPower > 1.0)
            rightPower = 1.0;
        if (rightPower < -1.0)
            rightPower = -1.0;
        if (leftPower > 1.0)
            leftPower = 1.0;
        if (leftPower < -1.0)
            leftPower = -1.0;

        if (m_closedLoopMode) {
            rightTalon1.set(ControlMode.Velocity, rightPower * m_maxWheelSpeed);
            leftTalon1.set(ControlMode.Velocity, leftPower * m_maxWheelSpeed);
        } else {
            rightTalon1.set(ControlMode.PercentOutput, rightPower);
            leftTalon1.set(ControlMode.PercentOutput, leftPower);
        }
    }

    /*
     * This method allows one to drive in "Tank Drive Mode". Tank drive mode uses
     * the left side of the joystick to control the left side of the robot, whereas
     * the right side of the joystick controls the right side of the robot.
     */
    public void tankDrive(double leftSideThrottle, double rightSideThrottle) {
        setMotorPower(leftSideThrottle, rightSideThrottle);
    }

    public double getPitch() {
        return headingCorrection.getPitch();
    }

    double m_lastThrottle;
    double m_lastSteering;
    double m_lastLeftPower;
    double m_lastRightPower;

    public void speedRacerDrive(double throttle, double rawSteeringX, boolean quickTurn) {
        double leftPower;
        double rightPower;
        double rotation = 0;
        final double QUICK_TURN_GAIN = 0.55; // 2019: .75. 2020: .75 was too fast.

        m_lastThrottle = throttle;
        m_lastSteering = rawSteeringX;

        // check for if steering input is essentially zero for "DriveStraight"
        // functionality
        if ((-0.01 < rawSteeringX) && (rawSteeringX < 0.01)) {
            // no turn being commanded, drive straight or stay still
            m_iterationsSinceRotationCommanded++;

            if ((-0.01 < throttle) && (throttle < 0.01)) {
                // System.out.println("Drive: stay still");

                // no motion commanded, stay still
                m_iterationsSinceMovementCommanded++;
                rotation = 0.0;

                // if we are standing still and we are turned, assume the new direction is the
                // correct direction
                headingCorrection.lockHeading();
            } else {
                // driving straight
                if ((m_iterationsSinceRotationCommanded == LOOPS_GYRO_DELAY)
                        || (m_iterationsSinceMovementCommanded >= LOOPS_GYRO_DELAY)) {
                    // exactly LOOPS_GYRO_DELAY iterations with no commanded turn,
                    // or haven't had movement commanded for longer than LOOPS_GYRO_DELAY,
                    // so we want to take steps to preserve our current heading hereafter

                    // get current heading as desired heading
                    headingCorrection.lockHeading();
                    rotation = 0.0;

                    System.out.println("Drive: drive straight LOCK");
                } else if (m_iterationsSinceRotationCommanded < LOOPS_GYRO_DELAY) {
                    // not long enough since we were last turning,
                    // just drive straight without special heading maintenance
                    rotation = 0.0;
                    System.out.println("Drive: drive straight");
                } else if (m_iterationsSinceRotationCommanded > LOOPS_GYRO_DELAY) {
                    // after more then LOOPS_GYRO_DELAY iterations since commanded turn,
                    // maintain the target heading
                    rotation = headingCorrection.maintainHeading();
                    System.out.println("Drive: drive straight w/ correction");
                }
                m_iterationsSinceMovementCommanded = 0;
            }
            // driveStraight code benefits from "spin" behavior when needed
            leftPower = throttle + rotation;
            rightPower = throttle - rotation;
        } else {
            // commanding a turn, reset iterationsSinceRotationCommanded
            m_iterationsSinceRotationCommanded = 0;
            m_iterationsSinceMovementCommanded = 0;
            if (quickTurn) {

                int throttleSign;
                if (throttle >= 0.0) {
                    throttleSign = 1;
                } else {
                    throttleSign = -1;
                }

                // want a high-rate turn (also allows "spin" behavior)
                // power to each wheel is a combination of the throttle and rotation
                rotation = rawSteeringX * throttleSign * QUICK_TURN_GAIN;
                leftPower = throttle + rotation;
                rightPower = throttle - rotation;
            } else {
                // want a standard rate turn (scaled by the throttle)
                if (rawSteeringX >= 0.0) {
                    // turning to the right, derate the right power by turn amount
                    // note that rawSteeringX is positive in this portion of the "if"
                    leftPower = throttle;
                    rightPower = throttle * (1.0 - Math.abs(rawSteeringX));
                } else {
                    // turning to the left, derate the left power by turn amount
                    // note that rawSteeringX is negative in this portion of the "if"
                    leftPower = throttle * (1.0 - Math.abs(rawSteeringX));
                    rightPower = throttle;
                }
            }
        }

        m_lastLeftPower = leftPower;
        m_lastRightPower = rightPower;

        setMotorPower(leftPower, rightPower);
    }

    public void rotateToHeading(final double desiredHeading) {
        headingCorrection.setDesiredHeading(desiredHeading);
    }

    // **********************************************DISPLAY****************************************************

    double convertTicksToMeters(double ticks) {
        return ticks * DISTANCE_PER_PULSE_IN_METERS;
    }

    @Override
    public void periodic() {
        headingCorrection.periodic();
        updateHistory();
        updateSmartDashboard();
        m_odometry.update(
                Rotation2d.fromDegrees(headingCorrection.getHeading()),
                convertTicksToMeters(leftTalon1.getSelectedSensorPosition()),
                convertTicksToMeters(rightTalon1.getSelectedSensorPosition()));
    }

    private void updateSmartDashboard() {
        headingCorrection.updateSmartDashboard();

        SmartDashboard.putBoolean("In Autonomous", DriverStation.isAutonomous());
        SmartDashboard.putNumber("Battery Voltage", RobotController.getBatteryVoltage());

        SmartDashboard.putNumber("Throttle", m_lastThrottle);
        SmartDashboard.putNumber("Steering", m_lastSteering);
        SmartDashboard.putNumber("Left Power", m_lastLeftPower);
        SmartDashboard.putNumber("Right Power", m_lastRightPower);

        int matchnumber = DriverStation.getMatchNumber();
        DriverStation.MatchType MatchType = DriverStation.getMatchType();
        SmartDashboard.putString("matchInfo", "" + MatchType + '_' + matchnumber);

        SmartDashboard.putNumber("Left 1 Encoder Counts", leftTalon1.getSelectedSensorPosition(0));
        SmartDashboard.putNumber("Right 1 Encoder Counts", rightTalon1.getSelectedSensorPosition(0));
        SmartDashboard.putNumber("Left 2 Encoder Counts", leftTalon2.getSelectedSensorPosition(0));
        SmartDashboard.putNumber("Right 2 Encoder Counts", rightTalon2.getSelectedSensorPosition(0));
        SmartDashboard.putNumber("Left 3 Encoder Counts", leftTalon3.getSelectedSensorPosition(0));
        SmartDashboard.putNumber("Right 3 Encoder Counts", rightTalon3.getSelectedSensorPosition(0));

        // Note: getSpeed() returns ticks per 0.1 seconds
        SmartDashboard.putNumber("Left Encoder Speed", leftTalon1.getSelectedSensorVelocity(0));
        SmartDashboard.putNumber("Right Encoder Speed", rightTalon1.getSelectedSensorVelocity(0));

        DifferentialDriveWheelSpeeds speeds = getWheelSpeeds();
        SmartDashboard.putNumber("Drive Left Speed m per s", speeds.leftMetersPerSecond);
        SmartDashboard.putNumber("Drive Right Speed m per s", speeds.rightMetersPerSecond);

        SmartDashboard.putNumber("Drive Left Volts", leftTalon1.getMotorOutputVoltage());
        SmartDashboard.putNumber("Drive Right Volts", rightTalon1.getMotorOutputVoltage());
        SmartDashboard.putNumber("Drive Left Percent", m_lastLeftPercent);
        SmartDashboard.putNumber("Drive Right Percent", m_lastRightPercent);
        Pose2d pose = getPose();
        SmartDashboard.putNumber("Drive Pose X", pose.getX());
        SmartDashboard.putNumber("Drive Pose Y", pose.getY());

        // To convert ticks per 0.1 seconds into feet per second
        // a - multiply be 10 (tenths of second per second)
        // b - divide by 12 (1 foot per 12 inches)
        // c - multiply by distance (in inches) per pulse
        SmartDashboard.putNumber("Left Speed (fps)",
                leftTalon1.getSelectedSensorVelocity(0) * 10 / 12 * DISTANCE_PER_PULSE_IN_INCHES);
        SmartDashboard.putNumber("Right Speed (fps)",
                rightTalon1.getSelectedSensorVelocity(0) * 10 / 12 * DISTANCE_PER_PULSE_IN_INCHES);

        SmartDashboard.putNumber("Left Talon Output Voltage", leftTalon1.getMotorOutputVoltage());
        SmartDashboard.putNumber("Right Talon Output Voltage", rightTalon1.getMotorOutputVoltage());

        SmartDashboard.putNumber("LT Falcon Supply Current", leftTalon1.getSupplyCurrent());
        SmartDashboard.putNumber("LF Falcon Supply Current", leftTalon2.getSupplyCurrent());
        SmartDashboard.putNumber("LB Falcon Supply Current", leftTalon3.getSupplyCurrent());
        SmartDashboard.putNumber("RT Falcon Supply Current", rightTalon1.getSupplyCurrent());
        SmartDashboard.putNumber("RF Falcon Supply Current", rightTalon2.getSupplyCurrent());
        SmartDashboard.putNumber("RB Falcon Supply Current", rightTalon3.getSupplyCurrent());

        SmartDashboard.putBoolean("Closed Loop Mode", m_closedLoopMode);
    }

    private static final double CAMERA_LAG = 0.150; // was .200; changing to .150 at CMP

    int historyCount;

    public void updateHistory() {
        final double now = Timer.getFPGATimestamp();
        headingHistory.add(now, headingCorrection.getHeading());

        SmartDashboard.putNumber("Drive: History", historyCount++);
    }

    public double getHeading() {
        return headingCorrection.getHeading();
    }

    public double getHeadingForCapturedImage() {
        final double now = Timer.getFPGATimestamp();
        final double indexTime = now - CAMERA_LAG;
        return headingHistory.getAzForTime(indexTime);
    }

    /**
     * Start a distance travel
     */
    public void saveInitialWheelDistance() {
        m_initialWheelDistance = (getLeftEncoder() + getRightEncoder()) / 2;
    }

    public double getWheelDistance() {
        final double dist = (getLeftEncoder() + getRightEncoder()) / 2;
        return dist - m_initialWheelDistance;
    }

    // NOTE the difference between rotateToHeading(...) and goToHeading(...)
    public void setDesiredHeading(final double desiredHeading) {
        headingCorrection.setDesiredHeading(desiredHeading);
        m_iterationsSinceRotationCommanded = LOOPS_GYRO_DELAY + 1;
        m_iterationsSinceMovementCommanded = 0;

        // reset the heading control loop for the new heading
        headingCorrection.resetAndEnableHeadingPID();
    }

    public void tankDriveVolts(double leftVolts, double rightVolts) {
        m_lastLeftPercent = leftVolts;
        m_lastRightPercent = rightVolts;

        leftTalon1.set(TalonSRXControlMode.PercentOutput, leftVolts);
        rightTalon1.set(TalonSRXControlMode.PercentOutput, rightVolts);
    }

    /**
     * Returns the currently-estimated pose of the robot.
     * 
     * @return The pose
     */
    public Pose2d getPose() {
        return m_odometry.getPoseMeters();
    }

    /**
     * Returns the current wheel speeds of the robot.
     * 
     * @return The current wheel speeds.
     */
    public DifferentialDriveWheelSpeeds getWheelSpeeds() {
        return new DifferentialDriveWheelSpeeds(
                convertTicksToMeters(leftTalon1.getSelectedSensorVelocity() * 10), // *10 because it returns ticks per
                                                                                   // 100ms
                convertTicksToMeters(rightTalon1.getSelectedSensorVelocity() * 10));
    }

    public void resetOdometry(Pose2d pose) {
        leftTalon1.setSelectedSensorPosition(0.0);
        rightTalon1.setSelectedSensorPosition(0.0);
        m_odometry.resetPosition(
                Rotation2d.fromDegrees(headingCorrection.getHeading()), 0.0,
                0.0, pose);
    }

}