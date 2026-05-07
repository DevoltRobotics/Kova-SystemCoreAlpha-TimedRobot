package first.robot.mechanism;

import java.util.function.DoubleSupplier;

import org.wpilib.command3.Command;
import org.wpilib.command3.Mechanism;
import org.wpilib.command3.Scheduler;
import org.wpilib.drive.MecanumDrive;
import org.wpilib.driverstation.Gamepad;
import org.wpilib.hardware.expansionhub.ExpansionHubMotor;
import org.wpilib.math.controller.PIDController;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.networktables.DoubleTopic;
import org.wpilib.networktables.NTSendable;
import org.wpilib.networktables.NTSendableBuilder;
import org.wpilib.networktables.StringTopic;
import org.wpilib.units.Units;
import org.wpilib.util.sendable.SendableRegistry;

import first.robot.driver.GoBildaPinpointDriver;

public class MecanumMechanism extends Mechanism implements NTSendable {

    ExpansionHubMotor frontLeft, rearLeft, frontRight, rearRight;

    MecanumDrive drive;
    GoBildaPinpointDriver pinpoint;

    Pose2d lastPose = new Pose2d();

    PIDController xController = new PIDController(0, 0, 0);
    PIDController yController = new PIDController(0, 0, 0);
    PIDController headingController = new PIDController(0, 0, 0);

    public MecanumMechanism(
            ExpansionHubMotor frontLeft,
            ExpansionHubMotor rearLeft,
            ExpansionHubMotor frontRight,
            ExpansionHubMotor rearRight,
            GoBildaPinpointDriver pinpoint) {
        this.frontLeft = frontLeft;
        this.rearLeft = rearLeft;
        this.frontRight = frontRight;
        this.rearRight = rearRight;

        drive = new MecanumDrive(
                frontLeft::setThrottle,
                rearLeft::setThrottle,
                frontRight::setThrottle,
                rearRight::setThrottle);
        this.pinpoint = pinpoint;

        Scheduler.getDefault().addPeriodic(() -> {
            pinpoint.update();

            lastPose = new Pose2d(
                    Units.Millimeters.of(pinpoint.getPosXMm()),
                    Units.Millimeters.of(pinpoint.getPosYMm()),
                    Rotation2d.fromRadians(pinpoint.getHeadingRad()));
        });

        SendableRegistry.add(xController, "MecanumMechanism/PID/X");
        SendableRegistry.add(yController, "MecanumMechanism/PID/Y");
        SendableRegistry.add(headingController, "MecanumMechanism/PID/Heading");
    }

    public void setPose(Pose2d pose) {
        pinpoint.setPosition(
                Units.Meters.of(pose.getX()).in(Units.Millimeters),
                Units.Meters.of(pose.getY()).in(Units.Millimeters),
                pose.getRotation().getRadians());
        lastPose = pose;
    }

    public Pose2d getPose() {
        return lastPose;
    }

    public Command driveCartesianCmd(DoubleSupplier x, DoubleSupplier y, DoubleSupplier rotation,
            boolean fieldCentric) {
        return run(co -> {
            // init
            while (true) {
                if (fieldCentric) {
                    Pose2d currentPose = getPose();
                    drive.driveCartesian(
                        x.getAsDouble(), 
                        y.getAsDouble(), 
                        rotation.getAsDouble(),
                        currentPose.getRotation());
                } else {
                    drive.driveCartesian(x.getAsDouble(), y.getAsDouble(), rotation.getAsDouble());
                }
                co.yield();
            }

            // en
        }).named("MecanumDriveCmd");
    }

    public Command driveCartesianCmd(Gamepad gamepad, boolean fieldCentric) {
        return driveCartesianCmd(() -> gamepad.getLeftY(), () -> -gamepad.getLeftX(), () -> -gamepad.getRightX(),
                fieldCentric);
    }

    public Command driveToPointCmd(Pose2d targetPose, double maxThrottle) {
        return run(co -> {
            double safeMaxThrottle = Math.clamp(maxThrottle, 0, 1);

            xController.setSetpoint(targetPose.getX());
            yController.setSetpoint(targetPose.getY());
            headingController.setSetpoint(targetPose.getRotation().getDegrees());

            do {
                Pose2d currentPose = getPose();

                double xOutput = xController.calculate(currentPose.getX()) * safeMaxThrottle;
                double yOutput = yController.calculate(currentPose.getY()) * safeMaxThrottle;
                double headingOutput = headingController.calculate(currentPose.getRotation().getDegrees()) * safeMaxThrottle;

                drive.driveCartesian(xOutput, yOutput, headingOutput, currentPose.getRotation());

                co.yield();
            } while (!xController.atSetpoint() || !yController.atSetpoint() || !headingController.atSetpoint());

            drive.driveCartesian(0, 0, 0);
        }).named("MecanumDriveToPointCmd");
    }

    @Override
    public void initSendable(NTSendableBuilder builder) {
        builder.setSmartDashboardType("MecanumMechanism");

        var pubX = new DoubleTopic(builder.getTopic("Pose/X_m")).publish();
        var pubY = new DoubleTopic(builder.getTopic("Pose/Y_m")).publish();
        var pubHeading = new DoubleTopic(builder.getTopic("Pose/Heading_deg")).publish();
        var pubFreq = new DoubleTopic(builder.getTopic("Pinpoint/Frequency_Hz")).publish();
        var pubStatus = new StringTopic(builder.getTopic("Pinpoint/Status")).publish();

        builder.addCloseable(pubX);
        builder.addCloseable(pubY);
        builder.addCloseable(pubHeading);
        builder.addCloseable(pubFreq);
        builder.addCloseable(pubStatus);

        builder.setUpdateTable(() -> {
            pubX.set(lastPose.getX());
            pubY.set(lastPose.getY());
            pubHeading.set(lastPose.getRotation().getDegrees());
            pubFreq.set(pinpoint.getFrequencyHz());
            pubStatus.set(pinpoint.getDeviceStatus().toString());
        });
    }
}