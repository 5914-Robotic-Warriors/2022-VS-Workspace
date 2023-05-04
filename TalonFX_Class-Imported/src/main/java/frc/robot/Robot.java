/////////////////////////////////////////////////////////////////////
//  File:  Robot.Java
////////////////////////////////////////////////////////////////////
//
//  Purpose:  The primary timed robot application used to demonstrate
//            methods for actively controlling the TalonFX motor
//            via use of it's integrated encoder in "velocity mode", i.e.,
//            use of the encoder counts per 100msec to control the
//            motor speed.  It is intended that this be used to
//            set the "shooter" ball velocity when the motor
//            "velocity" and wheel diameter are known.
//
//  Compiler:  Microsoft VS, java language.
//
//  Revison History:  Let's start with this version 2/15/2021
//
//
//  Remarks:  At this point there are several functions that can
//            effectively control a single motor.  Dual motor
//            capability will be developed in the future once the
//            second motor on the test is updated with the proper
//            firmware.
//
//            Problems encountered in autonomous:  In the present
//            mode a runable thread is created that is designed
//            to run the motors at fixed speed for 3 seconds
//            once stability (velocity at target within deadband).
//            As near as I can tell, the thread does not terminate
//            on it's own.  The thread does function though.
//
//            In teleop, it is intended to activate the motors
//            within a joystick button press and hold.  Issues 
//            that need to be addressed and tested are multiple
//            starts.  It would be beneficial if I had a joystick
//            attached to the testboard.
//
//            02/18/2021:  Multiple motor operation established 
//            in teleop and autonomous modes.
//
//            02/20/2021:  robot_v1.6-alpha.zip
//            A new thread that runs through a 
//            series of velocitys is developed.  All working
//            at this point.
//
//            03/03/2021:  robot_v1.8-alpha.zip  Both the 
//            shoot thread and analysis thread are working
//            correctly.  Table lookup of motor commands
//            via desired belt velocities is working.  The
//            table was determined using the analysis thread
//            and taking into consideration the overdrive
//            gearing of the motors responsible for firing
//            of the ball on the actual robot.
//
//            10/12/2021:  Program revisted, updated
//            TrajectoryComputations class.
//
//
/////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////




// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Joystick;

//import com.ctre.phoenix.motorcontrol.TalonFXControlMode;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {

  private static final String kDefaultAuto = "Default";
  private static final String kCustomAuto = "My Auto";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();
  private int init=1;
  private static int status=0;

  static Joystick Logic3D;

  static TalFX_Shooter lf_shoot;
  static TalFX_Shooter rf_shoot;
 
  static ShootThread shoot;
  static AnalysisThread analysis;
  static double velocity;

  static boolean shoot_thread = false;
  static boolean analysis_thread = true;

  //  You need to set the CAN ID's here for the TalonFX motors
  //  I am setting them to 1 and 0 respectively, you will need
  //  to change them for your specific application.
  int talonfx_can_id_1=5;
  int talonfx_can_id_2=6;

  int count;


   // This function is run when the robot is first started up and should be used for any
   // initialization code.
   
  @Override
  public void robotInit() {

    double velocity;

    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);
    SmartDashboard.putData("Auto choices", m_chooser);



    //  Create the two objects with appropriate inversion state
    lf_shoot = new TalFX_Shooter(talonfx_can_id_1,false);  //  non-inverted
    rf_shoot = new TalFX_Shooter(talonfx_can_id_2,true);   //  inverted

    Logic3D = new Joystick(0);

    //  Setup shoot parameters
    lf_shoot.setWheelDiameter(4.0);
    rf_shoot.setWheelDiameter(4.0);

    System.out.printf("Wheel Diameter = %.3f inches\n\n",lf_shoot.getWheelDiameter());

    lf_shoot.setTargetVelocity(60.0); 
    rf_shoot.setTargetVelocity(60.0); 

    velocity=lf_shoot.getTargetVelocity();

    System.out.printf("Target Velocity = %.3f ft/sec\n\n",velocity);

    //  compute motor velocity target for the desired ball velocity
    lf_shoot.target_speed=lf_shoot.computeMotorSpeed(velocity);
    rf_shoot.target_speed=rf_shoot.computeMotorSpeed(velocity);

    System.out.printf("target speed lf = %.1f counts/100msec\n",lf_shoot.target_speed);
    //  The minus sign here is just for the printout.  It is taken care of in
    //  the setVelocity( ... ) function.
    System.out.printf("target speed rf = %.1f counts/100msec\n",(-1*rf_shoot.target_speed));  //  inverted
  
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
    m_autoSelected = m_chooser.getSelected();
    // m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
    System.out.println("Auto selected: " + m_autoSelected);
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    switch (m_autoSelected) {
      case kCustomAuto:
        // Put custom auto code here
        break;
      case kDefaultAuto:
      default:
        // Put default auto code here
        break;
    }


    //  We call the thread once.  I envision that this thread could be expanded
    //  to fire all four motors.
    if(init==1)  {

      //  How do we combine all four motors in a single thread?  
      //  Note that a function within the TalFX_Shooter class
      //  that is a variant of the function used in teleop
      

      //  Two different threads.  The ShootThread is one that might
      //  used in autonomous during a competition.  The AnalysisThread
      //  runs the motors through a range of velocities to determine
      //  a table (in the future) that could provide lookup values
      //  for motor drive fraction verses belt velocity.
      //  Both threads work at this point.  They use a different
      //  version of setVelocity() than teloOp.

      if(shoot_thread==true)  {
          shoot=new ShootThread("shoot thread");
      }  else if(analysis_thread==true)  {
          analysis=new AnalysisThread("Analysis Thread");
      }

      count=0;
      init=0;

        
    }

    //  This is some debugging stuff that verifies the thread
    //  is alive or dead.  Works as expected.
    count++;

    if(shoot_thread==true)  {
      if(count==100)  {
          System.out.println("Thread Active = " + shoot.isactive);
          System.out.println("Thread Alive? = " + shoot.t.isAlive());
          count=0;
      }
    } else if(analysis_thread==true)  {
      if(count==100)  {
        System.out.println("Thread Active = " + analysis.isactive);
        System.out.println("Thread Alive? = " + analysis.t.isAlive());
        count=0;
      }
    }
    
  
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {}

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {    

    //  Telop should work OK with multiple motors and these class functions
    //  Introduce status as a means of turning the motor off after
    //  duration in seconds.
        
    //  Here we placed inside a block associated
    //  with a button press.  Multiple presses result in multiple
    //  runs.  The velocity is set within Robot.init()
    if (Logic3D.getRawButton(5) == true) {
        
          status=lf_shoot.setVelocity_teleOp();
          status=rf_shoot.setVelocity_teleOp();
       
    }  else  {

          lf_shoot.stop();
          rf_shoot.stop();
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

  public static int getStatus()  {
    return(status);

  }

}


