// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.drive.MecanumDrive;

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

  // Declare the Talons as WPI Talons so the MecanumDrive function accepts them
  WPI_TalonFX frontLeft;
  WPI_TalonFX frontRight;
  WPI_TalonFX backLeft;
  WPI_TalonFX backRight;

  static Joystick stick;

  MecanumDrive robotDrive;

  static Constants constants;
  static Climb climb;
  static Intake intake;

  // Counter to slow down position updates within TeleOp_Periodic
  int teleOpUpdateCount = 0;

  double frontLeft_position;
  double frontRight_position;
  double backLeft_position;
  double backRight_position;

  /**
   * This function is run when the robot is first started up and should be used
   * for any
   * initialization code.
   */
  @Override
  public void robotInit() {

    constants = new Constants();
    climb = new Climb();
    intake = new Intake();

    frontLeft = new WPI_TalonFX(constants.frontLeftDrive);
    frontRight = new WPI_TalonFX(constants.frontRightDrive);
    backLeft = new WPI_TalonFX(constants.backLeftDrive);
    backRight = new WPI_TalonFX(constants.backRightDrive);

    frontLeft.configFactoryDefault();
    frontRight.configFactoryDefault();
    backLeft.configFactoryDefault();
    backRight.configFactoryDefault();
    frontLeft.setNeutralMode(NeutralMode.Coast);
    frontRight.setNeutralMode(NeutralMode.Coast);
    backLeft.setNeutralMode(NeutralMode.Coast);
    backRight.setNeutralMode(NeutralMode.Coast);

    // Invert the right side motors because we need it to work with the magic
    // function
    frontRight.setInverted(true);
    backRight.setInverted(true);

    // Setup the internal encoders, 0 implies primary closed loop, 50msec timeout
    frontLeft.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 50);
    frontRight.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 50);
    backLeft.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 50);
    backRight.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 50);

    // Zero out the encoder positions
    frontLeft.setSelectedSensorPosition(0.0);
    frontRight.setSelectedSensorPosition(0.0);
    backLeft.setSelectedSensorPosition(0.0);
    backRight.setSelectedSensorPosition(0.0);

    // Create the MecanumDrive
    robotDrive = new MecanumDrive(frontLeft, backLeft, frontRight, backRight);

    stick = new Joystick(0);

  }

  @Override
  public void robotPeriodic() {
  }

  @Override
  public void autonomousInit() {
  }

  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopInit() {
  }

  @Override
  public void teleopPeriodic() {

    climb.climbGrab();
    climb.rotateClimb();
    climb.runLift();

    intake.Shoot();
    intake.intakeBall();
    intake.kickDown();
    

    // add the triggers into one axis so the driveCartesian program accepts it
    double triggerAddition;

    // divide by 2 in order to increase precision and eliminate getting -2 and 2 as
    // max values, instead getting -1 and 1
    triggerAddition = (-stick.getRawAxis(constants.Left_Trigger) / 2 + stick.getRawAxis(constants.Right_Trigger) / 2);

    // Use the second input for lateral movement, first input for forward
    // movement, and the third input for rotation.
    robotDrive.driveCartesian(-stick.getRawAxis(constants.Left_Y_Axis), triggerAddition,
        stick.getRawAxis(constants.Left_X_Axis), 0.0);
  }

  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
  }

  @Override
  public void testInit() {
  }

  @Override
  public void testPeriodic() {
  }

  @Override
  public void simulationInit() {
  }

  @Override
  public void simulationPeriodic() {
  }
}
