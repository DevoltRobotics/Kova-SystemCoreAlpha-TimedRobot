package first.robot.mechanism;

import java.util.function.DoubleSupplier;

import org.wpilib.command3.Command;
import org.wpilib.command3.Mechanism;
import org.wpilib.drive.MecanumDrive;
import org.wpilib.driverstation.Gamepad;
import org.wpilib.hardware.expansionhub.ExpansionHubMotor;
import org.wpilib.hardware.imu.OnboardIMU;

public class MecanumMechanism extends Mechanism {

    ExpansionHubMotor frontLeft, rearLeft, frontRight, rearRight;

    OnboardIMU imu;

    MecanumDrive drive;

    public MecanumMechanism(
        ExpansionHubMotor frontLeft, 
        ExpansionHubMotor rearLeft, 
        ExpansionHubMotor frontRight, 
        ExpansionHubMotor rearRight, 
        OnboardIMU imu
    ) {
        this.frontLeft = frontLeft;
        this.rearLeft = rearLeft;
        this.frontRight = frontRight;
        this.rearRight = rearRight;
        this.imu = imu;

        drive = new MecanumDrive(frontLeft::setThrottle, rearLeft::setThrottle, frontRight::setThrottle, rearRight::setThrottle);
    }

    public void driveCartesian(double x, double y, double rotation, boolean fieldCentric) {
        if(fieldCentric) {
            drive.driveCartesian(x, y, rotation, imu.getRotation2d());
        } else {
            drive.driveCartesian(x, y, rotation);
        }
    }

    public Command driveCartesianCmd(DoubleSupplier x, DoubleSupplier y, DoubleSupplier rotation, boolean fieldCentric) {
        return run(co -> {
            // init
            while(true) {
                driveCartesian(x.getAsDouble(), y.getAsDouble(), rotation.getAsDouble(), fieldCentric);
                co.yield();
            }

            // en
        }).named("MecanumDriveCmd");
    }

    public Command driveCartesianCmd(Gamepad gamepad, boolean fieldCentric) {
        return driveCartesianCmd(() -> gamepad.getLeftY(), () -> -gamepad.getLeftX(), () -> -gamepad.getRightX(), fieldCentric);
    }

    public void resetYaw() {
        imu.resetYaw();
    }

    public Command resetYawCmd() {
        return run(co -> {
            resetYaw();
            co.park();
        }).named("ResetYawCmd");
    }
    
}