// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the
 * name of this class or
 * the package after creating this project, you must also update the
 * build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private static final String auto1 = "Far Left 2 Ball";
  private static final String auto2 = "auto2";
  private String autoSelected;
  private final SendableChooser<String> autoChooser = new SendableChooser<>();

  static Joystick stick;

  static Constants constants;
  static Shooter shooter;
  static Climb climb;
  static Delay delay;
  static Drive drive;
  static Autonomous auto;

  boolean autoOnce;


  // WPI_TalonFX frontLeft;
  // WPI_TalonFX frontRight;
  // WPI_TalonFX backLeft;
  // WPI_TalonFX backRight;

  // MecanumDrive robotDrive;

  /**
   * This function is run when the robot is first started up and should be used
   * for any
   * initialization code.
   */
  @Override
  public void robotInit() {

    autoChooser.setDefaultOption("Wall Left 2 Ball", auto1);
    autoChooser.addOption("auto2", auto2);
    SmartDashboard.putData("Choose Autonomous", autoChooser);

    constants = new Constants();
    shooter = new Shooter();
    climb = new Climb();
    delay = new Delay();
    drive = new Drive();
    auto = new Autonomous();

    stick = new Joystick(0);

  }

  @Override
  public void robotPeriodic() {}

  @Override
  public void autonomousInit() {
    autoSelected = autoChooser.getSelected();
    // m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
    System.out.println("Auto Selected: " + autoSelected);

    drive.frontRight.setNeutralMode(NeutralMode.Brake);
    drive.backRight.setNeutralMode(NeutralMode.Brake);
    drive.frontLeft.setNeutralMode(NeutralMode.Brake);
    drive.backLeft.setNeutralMode(NeutralMode.Brake);

    autoOnce = true;
  }

  @Override
  public void autonomousPeriodic() {
    if (autoOnce == true) {
      switch (autoSelected) {
        case auto1:
          auto.wallLeft2Ball();
          break;
        case auto2:
        default:
          // put auto code here
          break;
      }
    }
    autoOnce = false;
    drive.backRight.setNeutralMode(NeutralMode.Coast);
    drive.backLeft.setNeutralMode(NeutralMode.Coast);
    drive.frontRight.setNeutralMode(NeutralMode.Coast);
    drive.frontLeft.setNeutralMode(NeutralMode.Coast);
  }

  @Override
  public void teleopInit() {
    drive.backRight.setNeutralMode(NeutralMode.Coast);
    drive.backLeft.setNeutralMode(NeutralMode.Coast);
    drive.frontRight.setNeutralMode(NeutralMode.Coast);
    drive.frontLeft.setNeutralMode(NeutralMode.Coast);
  }

  @Override
  public void teleopPeriodic() {
    
    climb.toggleClimbMode(constants.PS);

    if(climb.runClimbMode == true) {
    climb.rotateClimb();
    climb.climbGrab();
    }

    climb.runLift();

    if(climb.runClimbMode == false) {
    shooter.Intake();
    shooter.ShootLowPower();
    shooter.ShootHighPower();
    shooter.kickDown();
    }
    shooter.shooterMotorsOff();

    drive.invertDrive();
    drive.invertedDriveMotors();
    drive.standardDriveMotors();

  }

  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
  }

  @Override
  public void testInit() {
    drive.backRight.setNeutralMode(NeutralMode.Coast);
    drive.backLeft.setNeutralMode(NeutralMode.Coast);
    drive.frontRight.setNeutralMode(NeutralMode.Coast);
    drive.frontLeft.setNeutralMode(NeutralMode.Coast);
  }

  @Override
  public void testPeriodic() {
    double encoderPrint = drive.backRight.getSelectedSensorPosition();
    System.out.println("Encoder: " + encoderPrint);

    double gyroPrint = drive.driveGyro.getAngle();
    System.out.println("Gyro: " + gyroPrint);
  }

  @Override
  public void simulationInit() {
  }

  @Override
  public void simulationPeriodic() {
  }
}
