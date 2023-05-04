// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.drive.MecanumDrive;

/** This is a demo program showing how to use Mecanum control with the MecanumDrive class. */
public class Robot extends TimedRobot {
  WPI_TalonFX frontLeft;
  WPI_TalonFX frontRight;
  WPI_TalonFX backLeft;
  WPI_TalonFX backRight;

  Joystick stick;

  MecanumDrive robotDrive;

  @Override
  public void robotInit() {
    WPI_TalonFX frontLeft = new WPI_TalonFX(1);
    WPI_TalonFX frontRight = new WPI_TalonFX(2);
    WPI_TalonFX backLeft = new WPI_TalonFX(3);
    WPI_TalonFX backRight = new WPI_TalonFX(4);

    stick = new Joystick(0);

    // Invert the right side motors.
    // You may need to change or remove this to match your robot.
    frontRight.setInverted(true);
    backRight.setInverted(true);

    robotDrive = new MecanumDrive(frontLeft, backLeft, frontRight, backRight);
  }

  @Override
  public void teleopPeriodic() {
    //add the triggers into one axis so the driveCartesian program accepts it
    double triggerAddition;

    //divide by 2 in order to increase precision and eliminate getting -2 and 2 as max values, instead getting -1 and 1
    triggerAddition = (-stick.getRawAxis(3) / 2 + stick.getRawAxis(4) / 2);

    // Use the second input for lateral movement, first input for forward
    // movement, and the third input for rotation.
    robotDrive.driveCartesian(-stick.getRawAxis(1), triggerAddition, stick.getRawAxis(0), 0.0);
  }
}
