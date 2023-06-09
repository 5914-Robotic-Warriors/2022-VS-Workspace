// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  
WPI_TalonFX talon;
WPI_TalonFX talon2;
WPI_TalonFX talon3;

Joystick stick;

  /**`
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {

  talon = new WPI_TalonFX(6);
  talon2 = new WPI_TalonFX(7);
  talon3 = new WPI_TalonFX(5);
  talon2.setInverted(true);

  stick = new Joystick(0);

  talon.setNeutralMode(NeutralMode.Coast);
  talon2.setNeutralMode(NeutralMode.Coast);
  // neo.setIdleMode(IdleMode.kCoast);

  }

  @Override
  public void robotPeriodic() {}

  @Override
  public void autonomousInit() {}

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {}

  @Override
  public void teleopPeriodic() {
    
    talon.set(ControlMode.PercentOutput, -stick.getRawAxis(1));
    talon2.set(ControlMode.PercentOutput, -stick.getRawAxis(1));

    // neo.set(-stick.getRawAxis(5));

  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void testInit() {}

  @Override
  public void testPeriodic() {}

  @Override
  public void simulationInit() {}

  @Override
  public void simulationPeriodic() {}
}
