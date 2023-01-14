package frc.robot.commands;

// import org.mayheminc.util.MayhemOperatorPad;
import frc.robot.util.DriverPad;
import edu.wpi.first.wpilibj2.command.CommandBase;
// import frc.robot.RobotContainer;
// import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Drivebase;

public class DrivebaseTeleop extends CommandBase {
    private final Drivebase drive;
    // private final Climber climber;
    private final DriverPad driverStick;
    // private final MayhemOperatorPad operatorPad;

    // public DriveBaseTeleop(Drivebase drive, Climber climber, DriverPad
    // driverStick,
    // OperatorPad operatorPad) {
    public DrivebaseTeleop(Drivebase drive, DriverPad driverStick) {
        this.driverStick = driverStick;
        // this.operatorPad = operatorPad;

        this.drive = drive;
        // this.climber = climber;

        // addRequirements(drive);
        // addRequirements(climber);
    }

    @Override
    public void initialize() {
        this.drive.init();
        // RobotContainer.cameraLights.on();
    }

    @Override
    public void execute() {
        double throttle = this.driverStick.driveThrottle();
        double steering = this.driverStick.steeringX();
        boolean quickTurn = this.driverStick.quickTurn();

        this.drive.speedRacerDrive(throttle, steering, quickTurn);

        // double d = -operatorPad.OPERATOR_PAD.getY();
        // this.climber.setArmLengthPowerTo(d);
    }
}
