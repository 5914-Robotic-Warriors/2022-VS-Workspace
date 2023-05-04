// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {

  //Joystick
  private final Joystick stick = new Joystick(0);

  //Pneumatics IDs
  int sol1rev = 0;
  int sol1for = 1;
  int sol2rev = 3;
  int sol2for = 4;

  //Pneumatics
  private final Compressor comp = new Compressor(0, PneumaticsModuleType.CTREPCM);
  private final DoubleSolenoid sol1 = new DoubleSolenoid(0, PneumaticsModuleType.CTREPCM, sol1for, sol1rev);
  private final DoubleSolenoid sol2 = new DoubleSolenoid(0, PneumaticsModuleType.CTREPCM, sol2for, sol2rev);

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {}

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {}

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
  public void autonomousInit() {}

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
    comp.disable();
    
    sol1.set(Value.kReverse);
    sol2.set(Value.kReverse);
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    if (stick.getRawButton(1)){
      sol1.set(Value.kForward);
    } else if (stick.getRawButton(2)){
      sol1.set(Value.kReverse);
    }

    if (stick.getRawButton(4)){
      sol2.set(Value.kForward);
    } else if (stick.getRawButton(3)){
      sol2.set(Value.kReverse);
    }

    if (stick.getRawButton(6)){
      comp.enableDigital();
    } else if (stick.getRawButton(5)){
      comp.disable();
    }
  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {}

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {}

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {}

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}
