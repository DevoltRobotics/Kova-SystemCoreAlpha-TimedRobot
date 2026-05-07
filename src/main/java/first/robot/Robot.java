// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package first.robot;

import org.wpilib.command3.Scheduler;
import org.wpilib.command3.Trigger;
import org.wpilib.driverstation.Gamepad;
import org.wpilib.framework.TimedRobot;
import org.wpilib.hardware.accelerometer.ADXL345_I2C;
import org.wpilib.hardware.expansionhub.ExpansionHubMotor;
import org.wpilib.hardware.expansionhub.ExpansionHubServo;
import org.wpilib.hardware.imu.OnboardIMU;
import org.wpilib.hardware.imu.OnboardIMU.MountOrientation;
import org.wpilib.smartdashboard.SendableChooser;
import org.wpilib.smartdashboard.SmartDashboard;

import first.robot.mechanism.MecanumMechanism;

/**
 * The methods in this class are called automatically corresponding to each mode, as described in
 * the TimedRobot documentation. If you change the name of this class or the package after creating
 * this project, you must also update the Main.java file in the project.
 */
public class Robot extends TimedRobot {
  private static final String kDefaultAuto = "Default";
  private static final String kCustomAuto = "My Auto";
  private String autoSelected;
  private final SendableChooser<String> chooser = new SendableChooser<>();

  ExpansionHubMotor frontLeft = new ExpansionHubMotor(0, 0);
  ExpansionHubMotor rearLeft = new ExpansionHubMotor(0, 1);
  ExpansionHubMotor frontRight = new ExpansionHubMotor(0, 2);
  ExpansionHubMotor rearRight = new ExpansionHubMotor(0, 3);

  ExpansionHubServo claw = new ExpansionHubServo(0, 0);

  OnboardIMU imu = new OnboardIMU(MountOrientation.FLAT);

  MecanumMechanism mecanumSubsystem = new MecanumMechanism(frontLeft, rearLeft, frontRight, rearRight, imu);

  Gamepad gamepad1 = new Gamepad(0);

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  public Robot() {
    frontLeft.setReversed(true);
    rearLeft.setReversed(true);

    frontLeft.setFloatOn0(false);
    rearLeft.setFloatOn0(false);
    frontRight.setFloatOn0(false);
    rearRight.setFloatOn0(false);

    chooser.setDefaultOption("Default Auto", kDefaultAuto);
    chooser.addOption("My Auto", kCustomAuto);
    SmartDashboard.putData("Auto choices", chooser);
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and utility.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    Scheduler.getDefault().run();
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select between different
   * autonomous modes using the dashboard. The sendable chooser code works with the Java
   * SmartDashboard. If you prefer the LabVIEW Dashboard, remove all of the chooser code and
   * uncomment the getString line to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional comparisons to the switch structure
   * below with additional strings. If using the SendableChooser make sure to add them to the
   * chooser code above as well.
   */
  @Override
  public void autonomousInit() {
    autoSelected = chooser.getSelected();
    // autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
    System.out.println("Auto selected: " + autoSelected);
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    switch (autoSelected) {
      case kCustomAuto:
        // Put custom auto code here
        break;
      case kDefaultAuto:
      default:
        // Put default auto code here
        break;
    }
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
    Scheduler.getDefault().schedule(mecanumSubsystem.driveCartesianCmd(gamepad1, true));

    // Trigger resetFieldCentricTrigger = new Trigger(() -> gamepad1.getDpadUpButton());
    // resetFieldCentricTrigger.onTrue(mecanumSubsystem.resetYawCmd());
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    if(gamepad1.getDpadUpButtonPressed()) {
      imu.resetYaw();
    }
  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {}

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {}

  /** This function is called once when utility mode is enabled. */
  @Override
  public void utilityInit() {}

  /** This function is called periodically during utility mode. */
  @Override
  public void utilityPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}
