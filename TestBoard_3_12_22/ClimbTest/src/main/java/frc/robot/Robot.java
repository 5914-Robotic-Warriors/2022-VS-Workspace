// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {

//  Have to declare this as static if we are using the joystick
//  in files that extend Robot.
static Joystick stick;

Climb climb;

Shooter shooter;

Delay delay;

int update_counter=0;


  /**`
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {

    climb=new Climb();

    shooter=new Shooter();

    stick = new Joystick(0);

    delay=new Delay();

  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
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
  public void autonomousInit() {
    
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {

        double falcon_position;
        double neo_position;

       

        //  Zero encoders
        climb.falcon.setSelectedSensorPosition(0);
        shooter.neos_encoder.setPosition(0.0);

        delay.delay_milliseconds(20);

        falcon_position=climb.falcon.getSelectedSensorPosition();
        neo_position=shooter.neos_encoder.getPosition();

        System.out.printf("falcon_position = %.3f counts\n",falcon_position);
        System.out.printf("neo_position = %.3f revolutions\n",neo_position);
     
        //  Insure that the cylinder is in the proper starting
        //  position.
        climb.pneumo.enableDoubleSolenoidReverse();
        climb.state=climb.retracted;
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {

    //double neo_speed_error;
    //double falcon_position_error;
    //double falcon_speed_error;

    //climb.pneumo.compressorOff();  //  Off for testing
    climb.processCylinder(5);


    //  Here are some possible control mechanisms during teleOp for
    //  rotating the climb assembly.  Hold the button until the 
    //  movement is complete, letting go of the button re-inits
    //  the position for any subsequent movements.
    if(stick.getRawButton(3)==true)  {
        climb.rotate2Target(0.8,-90.0);
    }  else if(stick.getRawButton(3)==false){
        climb.position_init=1;
    }

    //  Holding the button down establishes a fixed speed
    //  regardless of external forces.  If a ball is fired
    //  and slows the shooter down, the shooter will regain
    //  the targeted speed as quickly as the settings allow.
    //  Releasing the button stops the shooter motor.
    if(stick.getRawButton(4)==true)  {
      shooter.setNEOspeed(3000.0);
    }  else if(stick.getRawButton(4)==false){
      shooter.setNEOoff();
    }

    //
    //neo_speed_error=shooter.setNEOspeed(3000.0);

    //falcon_position_error=climb.rotate2Target(0.8,-90.0);

    //falcon_speed_error=climb.setFALCONspeed(-3000.0);
    /*
      if(update_counter==10)  {
        //System.out.printf("NEO speed error = %.3f RPM\n",neo_speed_error);
        //System.out.printf("Falcon position error = %.3f counts\n",falcon_position_error);
        System.out.printf("Falcon speed error = %.3f RPM\n",falcon_speed_error);
        update_counter=0;
    }
    */
    update_counter++;
    

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
