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
  private static final String auto1 = "Left Wall 2 Ball";
  private static final String auto2 = "Left Wall 1 Ball";
  private static final String auto3 = "Middle Wall Right Side 2 Ball";
  private static final String auto4 = "Middle Wall Right Side 1 Ball";
  private static final String auto5 = "Right Wall 2 Ball";
  private static final String auto6 = "Right Wall 1 Ball";
  private static final String auto7 = "Move out of Tarmac Backwards";
  private static final String auto8 = "Move out of Tarmac Forwards";
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

    autoChooser.setDefaultOption("Left Wall 2 Ball", auto1);
    autoChooser.addOption("Wall Left 1 Ball", auto2);
    autoChooser.addOption("Middle Wall Right Side 2 Ball", auto3);
    autoChooser.addOption("Middle Wall Right Side 1 Ball", auto4);
    autoChooser.addOption("Right Wall 2 Ball", auto5);
    autoChooser.addOption("Right Wall 1 Ball", auto6);
    autoChooser.addOption("Move out of Tarmac Backwards", auto7);
    autoChooser.addOption("Move out of Tarmac Forwards", auto8);
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
  public void robotPeriodic() {
  }

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
          auto.wallLeft1Ball();
          break;
          case auto3:
          auto.middleWallRightSide2Ball();
          break;
          case auto4:
          auto.middleWallRightSide1Ball();
          break;
          case auto5:
          auto.rightWall2Ball();
          break;
          case auto6:
          auto.rightWall1Ball();
          break;
          case auto7:
          auto.driveOutOfAreaBwd();
          break;
          case auto8:
          auto.driveOutOfAreaFwd();
          break;
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

    if (climb.runClimbMode == true) {
      climb.rotateClimb();
      climb.climbGrab();
      climb.runLiftClimbMode();
      drive.invertDrive(constants.X);
      climb.toggleClimbMode(constants.Touchpad);
      drive.invertedDriveMotorsNoStrafe();
      drive.standardDriveMotorsNoStrafe();
      climb.climbAlign(constants.Options_Button);
    }

    if (climb.runClimbMode == false) {
      shooter.Intake();
      shooter.kickDown();
      shooter.ShootLowPower();
      shooter.ejectMotors();
      climb.turnClimbMotorsOff();
      climb.runLiftNormalMode();
      drive.invertDrive(constants.Right_Bumper);
      climb.toggleClimbMode(constants.Left_Bumper);
      drive.invertedDriveMotors();
      drive.standardDriveMotors();
      climb.climbAlign(constants.Options_Button);
    }

    climb.turnClimbMotorsOff();

    NormalizeEverything(constants.PS);

    shooter.shooterMotorsOff();

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

  public void NormalizeEverything(int button) {
    if (stick.getRawButton(button) == true) {
      climb.runClimbMode = false;
      drive.driveInverted = false;
    }
  }
}
