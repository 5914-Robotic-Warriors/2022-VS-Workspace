
/////////////////////////////////////////////////////////////////////
//  File:  TalFX_Shooter.java
/////////////////////////////////////////////////////////////////////
//
//  Purpose:  Provides construction of a TalonTX object with
//            member functions specific to operation of the shooter
//            mechanism of the 2021 robot project.
//
//  Compiler:  Java via Microsoft VS for FRC.  Libraries from Phoenix
//            and WPI are included.
//
//  Remarks:  02/04/2021:  Need to make more of the variables private
//            and add more access functions for those variables.
//
//            02/15/2021:  It would be beneficial to create a table
//            of motor drive percentages vs. either desired belt
//            velocity or encoder speed.  This would allow shorter
//            stabiliztion times.
//
//            One other possibility is that one encoder could be
//            used to stabilize all four motors.  It would be
//            necessary to establish that percentage output vs
//            rotational speed is consistent motor to motor.
//            Otherwise all motors will require feedback control.
//
//            02/20/21:  robot_v1.6-alpha.zip
//
//            Created versions of setVelocity() for teleOP and
//            autonomous versions.  It gets complex when motors
//            are operating in random directions.  Two motor
//            versions have been successfully created (outside
//            of this class).
//
//            10/12/2021: robot_v2.0-alpha.zip
//
//            Used format version of console output, i.e., printf( ... )
//            to avoid ridiculous number of decimal points.  Question
//            as to fractional display of digital counts. Will
//            investigate prior to zipping this code.
//
//
///////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////

package frc.robot;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;

public class TalFX_Shooter {

    private double wheel_diameter=4.0;    //  wheel diameter in inches

    //  There is a difference in the drive pulleys associated with
    //  the shooter motor and wheel.  Wheel: 24.23mm, FX:16.6mm
    private double overdrive=1.46;
    private double target_velocity;       //  target belt velocity ft/sec.
    private double starting_output=0.00;  //  motor output fraction
    private Boolean invert_state=false;   //  direction of motor rotation
    private Boolean once=true;
    private int init=1;
    private int count=0;

      //  Timing parame int  
    private long start_time=0;
    private long end_time=0;
    private double elapsed_time=0.0;

    TalonFX _talon;
    Delay delay;

    
    String str;

           
    /* String for output */
    StringBuilder _sb = new StringBuilder();
        
     int _loops = 0;             //  loop counter for printouts
    
    double target_speed;        //  target encoder speed in counts/100msec
    private double motor_output;        //  fractional motor output
    //private double output_increment;    //  output increment for motor output
    double deadband=50.0;        //  deadband in counts per 100msec

    //  Here we define and initialize a speed table.  Element '0' represents
    //  v=0, element '1' represents v=5ft/sec, '2' 10ft/sec and so on.  The
    //  values are empirically determined via the test board.  They are
    //  subject to change.  A linear interpolation will be used to 
    //  determine the initial motor command.
    //  
    double tbl_lookup;
  
    //  Base constructor
    TalFX_Shooter(int can_addr)  {

      //  Table generated via AnalysisThread
      //  Lists motor percentages every 5ft/sec
      //  with specified wheel and overdrive
      //  parameters.
      double tbl_lookup[] = new double[16];
        tbl_lookup[0]=0.0;
        tbl_lookup[1]=0.07;   //  5ft/sec
        tbl_lookup[2]=0.10;   //  10ft/sec
        tbl_lookup[3]=0.12;   //  15ft/sec
        tbl_lookup[4]=0.15;
        tbl_lookup[5]=0.18;
        tbl_lookup[6]=0.21;
        tbl_lookup[7]=0.24;
        tbl_lookup[8]=0.26;
        tbl_lookup[9]=0.29;
        tbl_lookup[10]=0.32;
        tbl_lookup[11]=0.35;
        tbl_lookup[12]=0.38;
        tbl_lookup[13]=0.41;
        tbl_lookup[14]=0.44;
        tbl_lookup[15]=0.47;
            
        /* Hardware */
        _talon = new TalonFX(can_addr);
        delay=new Delay();
            
        /* Factory Default all hardware to prevent unexpected behaviour */
        //_talon.configFactoryDefault();
        System.out.println("Error Code = " + _talon.configFactoryDefault());
            
        /* Config sensor used for Primary PID [Velocity] */
        //  Integrated Sensor (TalonFX has it), primary closed loop, 50 msec timeout
        _talon.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor,0, 50);
         
        //  configNominalOutputForward(double percentOutput,int timeoutMS)
        //  These calls indicate 0 output nominal
        _talon.configNominalOutputForward(0, 50);
        _talon.configNominalOutputReverse(0, 50);
        
        //  configPeakOutputForward(double percentOut,int timeoutMS)
        //  These calls indicate maximum output
        //  note sign reversal for reverse
        _talon.configPeakOutputForward(1,50);
        _talon.configPeakOutputReverse(-1,50);

        motor_output=starting_output;

        invert_state=false;

    
       
    }

    //  Constructor allowing specification of invert status.  Note that in
    //  the 2021 robot shooter configuration, two of the drive motors are
    //  inverted.
    TalFX_Shooter(int can_addr,Boolean invert)  {
          
        /* Hardware */
        _talon = new TalonFX(can_addr);
        delay=new Delay();
      
        /* Factory Default all hardware to prevent unexpected behaviour */
        //_talon.configFactoryDefault();
        System.out.println("Error Code = " + _talon.configFactoryDefault());
            
        /* Config sensor used for Primary PID [Velocity] */
        //  Integrated Sensor (TalonFX has it), primary closed loop, 50 msec timeout
        _talon.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor,0, 50);
            
    
        /* Config the peak and nominal outputs */
        // These functions are inherited from can.BaseMotorController
    
        //  configNominalOutputForward(double percentOutput,int timeoutMS)
        //  These calls indicate 0 output nominal
        _talon.configNominalOutputForward(0, 50);
        _talon.configNominalOutputReverse(0, 50);
        
        //  configPeakOutputForward(double percentOut,int timeoutMS)
        //  These calls indicate maximum output
        //  note sign reversal for reverse
        _talon.configPeakOutputForward(1,50);
        _talon.configPeakOutputReverse(-1,50);
              
        motor_output=starting_output;

        //  Set invert status
        if(invert==true)  {
          invert_state=true;
        }  else if(invert==false) {
          invert_state=false;
        }
            
    
    }


    //  Private data member access functions
    int setWheelDiameter(double diameter) {

        wheel_diameter=diameter;
        return(0);
    }

    double getWheelDiameter()  {

        return(wheel_diameter);
    }
    
    int setTargetVelocity(double velocity) {

        target_velocity=velocity;
        
        return(0);

    }

    double getTargetVelocity() {

        return(target_velocity);
    }

    int setStartingOutput(double output)
    {
        starting_output=output;
        return(0);
    }

    double getStartingOutput()
    {
        return(starting_output);
    }

    int setInvertState(Boolean state) 
    {
      invert_state=state;
      return(0);
    }

    Boolean getInvertState() 
    {
        return(invert_state);
    }

    int resetCount()
    {
      count=0;
      return(0);
    }

    int reInit()
    {
      init=1;
      return(0);
    }

  ///////////////////////////////////////////////////////////////////
  //  Function:  double computeMotorSpeed(double velocity)
  //////////////////////////////////////////////////////////////////
  //
  //  Purpose:  Given the desired shooter velocity in feet/second,
  //            based on the drive wheel diameter, it returns the
  //            desired rotational speed of the motor in encoder
  //            counts per 100msec (the direct readout of the
  //            encoder in velocity)
  //
  //  Arguments:Accepts the velocity in feet per second as double
  //
  //  Returns:  Motor speed in counts per 100msec as double
  //
  //  Remarks:  Uses the class member wheel_diameter.
  //
  ///////////////////////////////////////////////////////////////////
  ///////////////////////////////////////////////////////////////////
  public double computeMotorSpeed(double velocity)
  {
    int debug=0;
    double motor_rps;  //  motor revolutions per second
    double diameter;
    double counts_msec;

    diameter=wheel_diameter/12.0;  //  diameter in feet

    //  motor speed in revolutions per second
    //  velocity expressed in feet/second, diameter in feet
    motor_rps=velocity/(Math.PI*diameter);

    //  Here we account for the overdrive of 1.46:1
    //  For a given belt velocity, this will reduce the
    //  rps of the motor required.
    motor_rps/=overdrive;

    //  next, counts per 100 msec, 10 such intervals per second.
    counts_msec=2048.0*motor_rps/10.0;

    if(debug==1)  {
        System.out.printf("counts_msec = %.1f\n",counts_msec);
    }
  

    return(counts_msec);
  }

  //  Alternate overloaded function that allows changing of the
  //  wheel diameter.

  ///////////////////////////////////////////////////////////////////
  //  Function:  double computeMotorSpeed(double velocity, double diameter)
  //////////////////////////////////////////////////////////////////
  //
  //  Purpose:  Given the desired shooter velocity in meters/second,
  //            based on the drive wheel diameter, it returns the
  //            desired rotational speed of the motor in encoder
  //            counts per 100msec (the direct readout of the
  //            encoder in velocity)
  //
  //  Arguments:Accepts the velocity in feet per second as double
  //            Accepts the wheel diameter in inches.
  //
  //  Returns:  Motor speed in counts per 100msec as double
  //
  //  Remarks:  Allows specification of wheel_diameter in inches.
  //
  ///////////////////////////////////////////////////////////////////
  ///////////////////////////////////////////////////////////////////
  public double computeMotorSpeed(double velocity,double diameter)
  {
    int debug = 0;
    double motor_rps;  //  motor revolutions per second
    double counts_msec;

    //  convert wheel diameter to feet.
    diameter=diameter/12.0;  //  diameter in feet

    //  motor speed in revolutions per second
    //  Note important of operator precedence
    motor_rps=velocity/(Math.PI*diameter);

    //  Here we account for the overdrive of 1.46:1
    //  For a given belt velocity, this will reduce the
    //  rps of the motor required.
    motor_rps/=overdrive;

    //  next, coumpute counts per 100 msec.
    counts_msec=2048.0*motor_rps/10.0;

    if(debug==1) {
        System.out.printf("counts_msec = %.1f\n",counts_msec);
    }

    return(counts_msec);
  }

  ///////////////////////////////////////////////////////////////////
  // Function: int setVelocity_auto_()
  //////////////////////////////////////////////////////////////////
  //
  //  Purpose:  Establishes desired motor speed in counts/100msec.
  //
  //  Arguments:void
  //
  //  Returns:  Zero initially, returns '1' when stability at
  //            target_speed is established.  Once stability is
  //            established the motor runs at the last modified
  //            fractional input (-1,1)
  //
  //  Remarks:  This function is used in the autonomous mode for
  //            a single application of fixed duration (time).
  //            robot_v1.8-alpha.zip
  //
  ///////////////////////////////////////////////////////////////////
  ///////////////////////////////////////////////////////////////////
  public int setVelocity_auto_()
  {
    int debug=0;
    //  Temporary variables, calculated within this function
    double speed;
    double raw_error;
    double error;
    double incr;
  
    ///////////////////////////////////////////////////////////////////
    //  Start the timer, zero init, and reset the count the first time
    //  through.  Note that these (init and count) are private member variables
    //  and will be private to the class.
    //
    //  For multiple motors, each motor would be included within the
    //  block.
    //
    ////////////////////////////////////////////////////////////////////
    if(init==1)  {
        start_time=System.nanoTime();
        if(invert_state==true)  {
          motor_output*=-1.0;
          target_speed*=-1.0;
        }
        count=0;
        init=0;
    }

    //   First time through:
    //   Set Talon  output percentage at the starting point
    //   Future:  This could be read from a table to more quickly hit
    //   the target.  Table would relate velocity to starting
    //   point (percentage) for motor drive.
    

    //  First time through, this starts the motor.  Subsequent
    //  calls within teleop will alter the motor_output until
    //  measured speed satisfies deadband limits.
    _talon.set(TalonFXControlMode.PercentOutput, motor_output);
    

    //  Need this delay to allow motor to start before attempting
    //  to read the encoder velocity.  This is important, too short
    //  and motor has not responded to change in power.  This is a
    //  parameter that needs to be determined.
    delay.delay_milliseconds(50);

  

    //  Read the selected sensor velocity in counts per 100msec
    //  "0" implies primary closed loop, "1" impies auxiliary closed loop
    //  Essentially we are reading the integrated TalonFX sensor.
    //  If running in inverted state, speed will be negative
    speed=_talon.getSelectedSensorVelocity(0);
     
    
		
    
    //  Compute the difference error between target
    //  and actual motor speed
    raw_error=target_speed-speed;
    error=Math.abs(raw_error);

    if(debug==1)  {
      System.out.printf("raw error = %.1f  error = %.1f",raw_error,error);
    }

    //  Here we examine the stability of the motor speed with respect
    //  to the target.  "count" was initialized to zero in the init block.
    if(error<50.0)  {
        if(count<3) {
          count++;
          System.out.printf("raw error = %.1f  error = %.1f",raw_error,error);
        }  
    }

    if(count>=3)  {  //  consider output stable, do not alter motor_output
      end_time=System.nanoTime();  //  This is in nanoseconds
      elapsed_time=(double)((end_time-start_time)/1e6);  //  msec
      System.out.printf("Time to stability = %.3f msec\n",elapsed_time);
      System.out.printf("Error = %.1f counts/100msec\n",raw_error);
      System.out.printf("motor output = %.3f\n",motor_output);
      return(1);
  
    }  
   
    //  Here is our proportional error correction block
    //  These numbers determine many things:
    //  1.  Speed of correction (convergence).
    //  2.  Amount of "ringing"
    //  The goal is to get to our speed as quickly
    //  as possible with a minimum of "overshoot"
    //  You will need to play with these parameters
    if(error>2000)  {
      incr=0.05;
    } else if(error>1000.0)  {
      incr=0.02;
    } else if(error>500.0)  {
      incr=0.01;
    } else if(error>100.0)  {
      incr=0.002;
    }  else if(error>50.0) {
      incr=0.001;
    }  else  {  //  do nothing
      incr=0.0;
    }

    //  If we are within +/- deadband,
    //  do nothing.  Note that if we are operating
    //  in reverse (negative motor percentages)
    //  This block needs to be know.  With
    //  multiple motors perhaps the invert function
    //  will simplify this issue.  To do:  widen
    //  the deadband to determine the expected
    //  noise in the reading of the motor speed
    //  with a constant input.
    
    if(invert_state==false)  {
      if(speed<(target_speed-deadband))  {
        motor_output+=incr;
      } 
      else if(speed>(target_speed+deadband))  {
        motor_output-=incr;
      }  else  {
        ;
      }
    }  else {
      if(speed<(target_speed-deadband))  {
        motor_output+=incr;
      } 
      else if(speed>(target_speed+deadband))  {
        motor_output-=incr;
      }  else  {
        ;
      }

    }
    
    return(0);
  }



  ///////////////////////////////////////////////////////////////////
  // Function: int setVelocity_auto()
  //////////////////////////////////////////////////////////////////
  //
  //  Purpose:  Establishes desired motor speed in counts/100msec.
  //
  //  Arguments:void
  //
  //  Returns:  Zero initially, returns '1' when stability at
  //            target_speed is established.  Once stability is
  //            established the motor runs at the last modified
  //            fractional input (-1,1)
  //
  //  Remarks:  To be used within Autonomous, called approximately every 20msec
  //            Note that a separate instance of the class is 
  //            created for each motor with it's own set of
  //            member variables.
  //
  //            
  //            02/20/2021:  robot_v1.6.zip
  //
  //            On multiple calls, the inverted motor was 
  //            converging slower than the non-inverted motor. It
  //            was reversing direction. This has been corrected. 
  //            Use of the "once" boolean corrected this behavior.
  //            Renamed the function _auto() for autonomous
  //            operation.
  //
  ///////////////////////////////////////////////////////////////////
  ///////////////////////////////////////////////////////////////////
  public int setVelocity_auto()
  {
    int debug=0;
    //  Temporary variables, calculated within this function
    double speed;
    double raw_error;
    double error;
    double incr;
  
    ///////////////////////////////////////////////////////////////////
    //  Start the timer, zero init, and reset the count the first time
    //  through.  Note that these (init and count) are private member variables
    //  and will be private to the class.
    //
    //  For multiple motors, each motor would be included within the
    //  block.
    //
    ////////////////////////////////////////////////////////////////////
    if(init==1)  {
        start_time=System.nanoTime();
        if(once==true)  {
            if(invert_state==true)  {
              motor_output*=-1.0;
            }
            once=false;
        }
        if(invert_state==true) {
            target_speed*=-1.0;
        }
        count=0;
        init=0;
    }
    if(debug==1)  {
        System.out.printf("motoroutput = %.3f  target speed = %.3f ft/sec\n",motor_output,target_speed);
    }

    //   First time through:
    //   Set Talon  output percentage at the starting point
    //   Future:  This could be read from a table to more quickly hit
    //   the target.  Table would relate velocity to starting
    //   point (percentage) for motor drive.

    //   This function works well for a single power setting like in
    //   ShootThread or in a multiple power application such as
    //   AnalysiThread.

    //  First time through, this starts the motor.  Subsequent
    //  calls within autonomous will alter the motor_output until
    //  measured speed satisfies deadband limits.
    _talon.set(TalonFXControlMode.PercentOutput, motor_output);
   
    //  Need this delay to allow motor to start before attempting
    //  to read the encoder velocity.  This is important, too short
    //  and motor has not responded to change in power.  This is a
    //  parameter that needs to be determined.
    delay.delay_milliseconds(50);

  

    //  Read the selected sensor velocity in counts per 100msec
    //  "0" implies primary closed loop, "1" impies auxiliary closed loop
    //  Essentially we are reading the integrated TalonFX sensor.
    //  If running in inverted state, speed will be negative
    speed=_talon.getSelectedSensorVelocity(0);
     
    
		
    
    //  Compute the difference error between target
    //  and actual motor speed
    raw_error=target_speed-speed;
    error=Math.abs(raw_error);

    if(debug==1)  {
      System.out.printf("raw error = %.1f error = %.1f\n",raw_error,error);
    }

    //  Here we examine the stability of the motor speed with respect
    //  to the target.  "count" was initialized to zero in the init block.
    //if(error<50.0)  {
    if(error<deadband)  {
        if(count<3) {
          count++;
          if(debug==1)  {
            System.out.printf("raw error = %.1f error = %.1f\n",raw_error,error);
          }
        }  
    }

    if(count>=3)  {  //  consider output stable, do not alter motor_output
      end_time=System.nanoTime();  //  This is in nanoseconds
      elapsed_time=(double)((end_time-start_time)/1e6);  //  msec
      if(debug==1)  {
        System.out.printf("Time to stability = %.3f msec\n",elapsed_time);
        System.out.printf("Error = %.1f counts/100msec\n",raw_error);
        System.out.printf("motor output = %.3f\n",motor_output);
      }

      System.out.printf("Velocity = %.3f ft/sec  motor cmd = %.3f\n",Robot.analysis.v,motor_output);
      return(1);
  
    }  
   
    //  Here is our proportional error correction block
    //  These numbers determine many things:
    //  1.  Speed of correction (convergence).
    //  2.  Amount of "ringing"
    //  The goal is to get to our speed as quickly
    //  as possible with a minimum of "overshoot"
    //  You will need to play with these parameters
    if(error>2000)  {
      incr=0.05;
    } else if(error>1000.0)  {
      incr=0.02;
    } else if(error>500.0)  {
      incr=0.01;
    } else if(error>100.0)  {
      incr=0.002;
    }  else if(error>50.0) {
      incr=0.001;
    }  else  {  //  do nothing
      incr=0.0;
    }

    //  If we are within +/- deadband,
    //  do nothing.  Note that if we are operating
    //  in reverse (negative motor percentages)
    //  This block needs to know.  With
    //  multiple motors perhaps the invert function
    //  will simplify this issue.  To do:  widen
    //  the deadband to determine the expected
    //  noise in the reading of the motor speed
    //  with a constant input.
    
    if(invert_state==false)  {
      if(speed<(target_speed-deadband))  {
        motor_output+=incr;
      } 
      else if(speed>(target_speed+deadband))  {
        motor_output-=incr;
      }  else  {
        ;
      }
    }  else {
      if(speed<(target_speed-deadband))  {
        motor_output+=incr;
      } 
      else if(speed>(target_speed+deadband))  {
        motor_output-=incr;
      }  else  {
        ;
      }

    }
    
    return(0);
  }

  ///////////////////////////////////////////////////////////////////
  // Function: int setVelocity_teleop()
  //////////////////////////////////////////////////////////////////
  //
  //  Purpose:  Establishes desired motor speed in counts/100msec.
  //
  //  Arguments:void
  //
  //  Returns:  Zero initially, returns '1' when stability at
  //            target_speed is established.  Once stability is
  //            established the motor runs at the last modified
  //            fractional input (-1,1)
  //
  //  Remarks:  To be used within TeleOp, called approximately every 20msec
  //            Note that a separate instance of the class is 
  //            created for each motor with it's own set of
  //            member variables.
  //
  //            
  //
  //            02/20/2021:  robot_v1.6-alpha.zip
  //
  //            Works via joystick button teloOp
  //
  ///////////////////////////////////////////////////////////////////
  ///////////////////////////////////////////////////////////////////
  public int setVelocity_teleOp()
  {
    int debug=0;
    //  Temporary variables, calculated within this function
    double speed;
    double raw_error;
    double error;
    double incr;
  
    ///////////////////////////////////////////////////////////////////
    //  Start the timer, zero init, and reset the count the first time
    //  through.  Note that these (init and count) are private member variables
    //  and will be private to the class.
    //  Question:  How do we account for multiple calls?  It is expected
    //  that this function will be called multiple times from a joystick
    //  button.  Each time the button is pressed init and count will have
    //  to be reset.  So in Robot.java, teleopPeriodic() the code would 
    //  look something like this:
    //
    //  public void teleopPeriodic() { 
    //      if(button_held)  {
    //          lf_shoot.reInit();
    //          lf_shoot.resetCount();
    //          status=lf_shoot.setVelocity();
    //      } else  {
    //          //shut the motor off
    //          lf_shoot.stop();
    //      }
    //  }
    //
    //  For multiple motors, each motor would be included within the
    //  block.
    //
    ////////////////////////////////////////////////////////////////////
    if(init==1)  {
        start_time=System.nanoTime();
        
        if(invert_state==true)  {
          motor_output*=-1.0;
          target_speed*=-1.0;
        }
      
        count=0;
        init=0;
    }

    if(debug==1)  {
      System.out.printf("motoroutput = %.3f  target speed = %.3f ft/sec\n",motor_output,target_speed);
    }

    //   First time through:
    //   Set Talon  output percentage at the starting point
    //   Future:  This could be read from a table to more quickly hit
    //   the target.  Table would relate velocity to starting
    //   point (percentage) for motor drive.

    //   This function works well for a single power setting like in
    //   ShootThread or in Teleop.

    //  First time through, this starts the motor.  Subsequent
    //  calls within teleop will alter the motor_output until
    //  measured speed satisfies deadband limits.
    _talon.set(TalonFXControlMode.PercentOutput, motor_output);
    if(debug==1)  {
      System.out.printf("Motor Output = %.3f\n",motor_output);

    }

    //  Need this delay to allow motor to start before attempting
    //  to read the encoder velocity.  This is important, too short
    //  and motor has not responded to change in power.  This is a
    //  parameter that needs to be determined.
    delay.delay_milliseconds(50);

  

    //  Read the selected sensor velocity in counts per 100msec
    //  "0" implies primary closed loop, "1" impies auxiliary closed loop
    //  Essentially we are reading the integrated TalonFX sensor.
    //  If running in inverted state, speed will be negative
    speed=_talon.getSelectedSensorVelocity(0);
    
    
    //  Compute the difference error between target
    //  and actual motor speed
    raw_error=target_speed-speed;
    error=Math.abs(raw_error);

    if(debug==1)  {
      System.out.printf("raw error = %.1f error = %.1f\n",raw_error,error);
    }

    //  Here we examine the stability of the motor speed with respect
    //  to the target.  "count" was initialized to zero in the init block.
    //  02/23/2021:  error<50.0 should be error<deadband
    if(error<deadband)  {
        if(count<3) {
          count++;
          if(debug==1)  {
            System.out.printf("raw error = %.1f error = %.1f\n",raw_error,error);
          }
        }  
    }

    if(count>=3)  {  //  consider output stable, do not alter motor_output
      end_time=System.nanoTime();  //  This is in nanoseconds
      elapsed_time=(double)((end_time-start_time)/1e6);  //  msec
      if(debug==1) {
        System.out.printf("Time to stability = %.3f msec\n",elapsed_time);
        System.out.printf("Error = %.1f counts/100msec\n",raw_error);
        System.out.printf("motor output = %.3f\n",motor_output);
      }
      System.out.printf("Velocity = %.3f ft/sec  motor cmd = %.3f\n",Robot.analysis.v,motor_output);
      return(1);
  
    }  
   
    //  Here is our proportional error correction block
    //  These numbers determine many things:
    //  1.  Speed of correction (convergence).
    //  2.  Amount of "ringing"
    //  The goal is to get to our speed as quickly
    //  as possible with a minimum of "overshoot"
    //  You will need to play with these parameters
    if(error>2000)  {
      incr=0.05;
    } else if(error>1000.0)  {
      incr=0.02;
    } else if(error>500.0)  {
      incr=0.01;
    } else if(error>100.0)  {
      incr=0.002;
    }  else if(error>50.0) {
      incr=0.001;
    }  else  {  //  do nothing
      incr=0.0;
    }

    //  If we are within +/- deadband,
    //  do nothing.  Note that if we are operating
    //  in reverse (negative motor percentages)
    //  This block needs to be know.  With
    //  multiple motors perhaps the invert function
    //  will simplify this issue.  To do:  widen
    //  the deadband to determine the expected
    //  noise in the reading of the motor speed
    //  with a constant input.
    
    if(invert_state==false)  {
      if(speed<(target_speed-deadband))  {
        motor_output+=incr;
      } 
      else if(speed>(target_speed+deadband))  {
        motor_output-=incr;
      }  else  {
        ;
      }
    }  else {
      if(speed<(target_speed-deadband))  {
        motor_output+=incr;
      } 
      else if(speed>(target_speed+deadband))  {
        motor_output-=incr;
      }  else  {
        ;
      }

    }
    
    return(0);
  }

  

  ///////////////////////////////////////////////////////////////////
  //  Function: int stop()
  //////////////////////////////////////////////////////////////////
  //
  //  Purpose:  stops the motor
  //
  //  Arguments: void
  //
  //  Returns:  zero
  //
  //  Remarks:
  //
  ///////////////////////////////////////////////////////////////////
  ///////////////////////////////////////////////////////////////////
  public int stop()
  {
     _talon.set(TalonFXControlMode.PercentOutput,0.0);
    
     reInit();
     resetCount();
     
     return(0);
  }

   ///////////////////////////////////////////////////////////////////
  //  Function:
  //////////////////////////////////////////////////////////////////
  //
  //  Purpose:
  //
  //  Arguments:
  //
  //  Returns:
  //
  //  Remarks:
  //
  ///////////////////////////////////////////////////////////////////
  ///////////////////////////////////////////////////////////////////
    
}
