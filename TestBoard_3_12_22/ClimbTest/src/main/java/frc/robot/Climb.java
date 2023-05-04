
/////////////////////////////////////////////////////////////////////
//  File:  Climb.java
/////////////////////////////////////////////////////////////////////
//
//  Purpose:  This class is intended to encapsulate those devices
//            used for the climbing operation.  The number of 
//            components is reduced to comform with the devices
//            available on the test board, e.g.,
//
//            *  A Falcon brushless motor
//            *  A NEO brushless motor
//            *  An air compressor
//            *  A double solenoid
//            *  A double acting air cylinder
//
//            The climb uses a Falcon motor.
//
//  Inception:  3/12/2022
//
//              3/13/2022:  Completed angular positioning function
//              assuming the 200:1 gear reduction.  Works for both
//              positive and negative rotations.  In this application
//              (motors without load) it is accurate to within 1 degree.
//
//
//////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////

package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;


public class Climb extends Robot {

    pneumatics pneumo;
    WPI_TalonFX falcon;

    Delay delay;

    int state;
    final int extended = 1;
    final int retracted =2;

    Boolean button_enabled=true;

    int update_counter=0;
    double falcon_speed;
    double falcon_position;
 

    //  Play with this if you want to shorten time to obtain
    //  targeted speed.  For 3000 RPM, 0.5 works well as a
    //  starting point.
    double falcon_power=0.5;

    double rotation_target=0.0;

  
    private final double FALCON_deadband=25;

    int falcon_init=1;

    int position_init=1;

    //  class constructor
    Climb() {

        falcon = new WPI_TalonFX(6);

        falcon.setNeutralMode(NeutralMode.Brake);

        falcon.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor,0, 50);

        pneumo=new pneumatics();

        delay=new Delay();
    }

  /////////////////////////////////////////////////////////////////////
  //  Function:  int processCylinder(int button)
  /////////////////////////////////////////////////////////////////////
  //
  //  Purpose:  Processes the dual solenoid/air cylinder using a 
  //            single button.
  //
  //  Arguments:The joystick button number.
  //
  //  Returns:  Zero
  //
  //  Remarks:
  //
  //
  /////////////////////////////////////////////////////////////////////
  /////////////////////////////////////////////////////////////////////
  public int processCylinder(int button) 
  {
    if((Robot.stick.getRawButton(button)==true)&&(state==retracted)&&(button_enabled==true))  {
      pneumo.enableDoubleSolenoidForward();
      state=extended;
      button_enabled=false;
      System.out.println("Button Pressed, State = " + state);
      delay.delay_milliseconds(20);
    }  else if((Robot.stick.getRawButton(button)==true)&&(state==extended)&&(button_enabled==true)) {
      System.out.println("Button Pressed");
      pneumo.enableDoubleSolenoidReverse();
      state=retracted;
      button_enabled=false;
      System.out.println("Button Pressed, State = " + state);
      delay.delay_milliseconds(20);
    }  
    else if((Robot.stick.getRawButton(button)==false)  && (button_enabled==false)) {
      System.out.println("Button Released, State = " + state);
      button_enabled=true;
      delay.delay_milliseconds(20);
    } 

    return(0);
  }

 


    /////////////////////////////////////////////////////////////////
    //  Function:  double setFALCONspeed(double target)
    /////////////////////////////////////////////////////////////////
    //
    //  Purpose:  Sets the RPM of the Falcon motor.
    //
    //  Arguments:Accepts a double representing the target RPM
    //
    //  Returns: The error as double in RPM
    //
    //  Remarks:  The Falcon encoder returns counts per 100msec
    //            when read for velocity.  This must be converted
    //            to RPM in this function for comparison to the
    //            submitted target.
    //
    //  3/8/22:  Tested appears to work.  Refinement would be
    //           to adjust the power increment depending on the
    //           size of the calculated error.
    //  3/9/22:  Proportional power adjustment and 20msec delay
    //           appears to work.
    //  3/13/22: Tested for both positive and negative targets.
    //
    /////////////////////////////////////////////////////////////////
    /////////////////////////////////////////////////////////////////
    public double setFALCONspeed(double target)
    {
        int debug=0;
        double error;
        double speed;
     
        if(falcon_init==1) {
          if(target<0.0)falcon_power*=-1.0;
          falcon.set(ControlMode.PercentOutput, falcon_power);
          falcon_init=0;
        }
        delay.delay_milliseconds(20.0);
        speed=falcon.getSelectedSensorVelocity(0);
        speed=convert2RPM(speed);
        error=target-speed;

        if(Math.abs(error)>FALCON_deadband)  {
            error=target-speed;
            if(error>0.0)  {
              if(error>300.0) {
                falcon_power+=0.02;
              }  else if((error>100.0)&&(error<300.0)) {
                falcon_power+=0.01;
              }  else if((error>FALCON_deadband)&&(error<100.0)) {
                falcon_power+=0.005;
              }  else {
                ;
              }
           
            } else if(error<0.0)  {
              if(error<-300.0) {
                falcon_power-=0.02;
              }  else if((error<-100.0)&&(error>-300.0)) {
                falcon_power-=0.01;
              }  else if((error<-FALCON_deadband)&&(error>-100.0)) {
                falcon_power-=0.005;
              }  else {
                ;
              }
            }
            falcon.set(ControlMode.PercentOutput, falcon_power);
            delay.delay_milliseconds(20.0);
            speed=falcon.getSelectedSensorVelocity(0);
            speed=convert2RPM(speed); 
        }  else  {
          falcon.set(ControlMode.PercentOutput, falcon_power);
          delay.delay_milliseconds(20.0);
          speed=falcon.getSelectedSensorVelocity(0);
          speed=convert2RPM(speed);   
        }

        if(debug==1) {
          System.out.printf("Speed = %.3f RPM  ",speed);
          System.out.printf("Power = %.3f\n",falcon_power);
        }
        return(error);
    }
    

    /////////////////////////////////////////////////////////////////
    //  Function:  public int rotate2Target(double power,double degrees)
    /////////////////////////////////////////////////////////////////
    //
    //  Purpose:  Rotates the grabber assembly to the desired
    //            position in degrees.
    //
    //  Arguments:double power: fractional power for the motors();
    //            Note that this test board version has only
    //            one motor.  The leader/follower should be 
    //            implemented on the actual robot.  The power
    //            is assumed to be a positive number.  The sign
    //            of operations is determined by the sign of 
    //            the second argument.  Power is taken care of
    //            internally.
    //
    //  Returns:  A double representing the position error in counts.
    //
    //  Remarks:  It is assumed that during the rotation phase there
    //            will be significant loading that will allow
    //            (with the intense gear reduction) a single
    //            stop command to position.
    //            3/13/2022:  That said, with the motor free
    //            running and in brake mode a 90 degree rotation
    //            results in an error of ~-500 counts.  With the
    //            intended gear reduction of 200:1 this equates
    //            to an error of 0.4 degrees.
    //
    /////////////////////////////////////////////////////////////////
    /////////////////////////////////////////////////////////////////
    public double rotate2Target(double power,double degrees)
    {
      double error=0;
      double position;  //  position in counts

      if(position_init==1) {
          rotation_target=convert2Counts(degrees);
          falcon.setSelectedSensorPosition(0);
          System.out.printf("rotation_target = %.3f counts",rotation_target);
          if(degrees>0.0)falcon_power=power;
          else falcon_power*=-1.0;
          position_init=0;
          
      }

      //  Here we adjust for the sign of degrees

      if(degrees>0.0)  {

        falcon.set(ControlMode.PercentOutput,falcon_power);
        delay.delay_milliseconds(20);
        position=falcon.getSelectedSensorPosition(0);

        //  One would expect that the error will always
        //  be positive.  Each time we ask for another
        //  rotation we must reset position_init=1
        error=rotation_target-position;

        if(error>FALCON_deadband)  {
            if(error>10000.0) {
              falcon_power=0.4;
            }  else if((error>1000.0)&&(error<10000.0)) {
              falcon_power=0.2;
            }  else if((error>FALCON_deadband)&&(error<1000.0)) {
              falcon_power=0.1;
            }  else if(error<0.0)  {  //  Stop the motor
              falcon_power=0.0;
              falcon.set(ControlMode.PercentOutput,falcon_power);
            }
        }  else  {
          falcon_power=0.0;
          falcon.set(ControlMode.PercentOutput,falcon_power);
        }
     }  else if (degrees<0.0) {
        falcon.set(ControlMode.PercentOutput,falcon_power);
        delay.delay_milliseconds(20);
        position=falcon.getSelectedSensorPosition(0);
        //System.out.printf("position = %.3f counts\n",position);

        //  One would expect that the error will always
        //  start positive.  
        //  As position decreases it will approach zero.
        error=position-rotation_target;
        //System.out.printf("error = %.3f counts\n",error);

        if(error>FALCON_deadband)  {
            if(error>10000.0) {
              falcon_power=-0.4;
            }  else if((error>1000.0)&&(error<10000.0)) {
              falcon_power=-0.2;
            }  else if((error>FALCON_deadband)&&(error<1000.0)) {
              falcon_power=-0.1;
            }  else if(error<0.0)  {  //  Stop the motor
              falcon_power=0.0;
              falcon.set(ControlMode.PercentOutput,falcon_power);
            }
        }  else  {
          falcon_power=0.0;
          falcon.set(ControlMode.PercentOutput,falcon_power);
        }

    }
    return(error);
  }


    /////////////////////////////////////////////////////////////////
    //  Function:  double convert2RPM(double)
    /////////////////////////////////////////////////////////////////
    //
    //  Purpose:
    //
    //  Arguments:
    //
    //  Returns:
    //
    //  Remarks:
    //
    /////////////////////////////////////////////////////////////////
    /////////////////////////////////////////////////////////////////
    private double convert2RPM(double speed)
    {
        double rpm;

        speed*=10.0;   //  Convert counts/100msec to counts/sec
        speed/=2048.0; //  Convert to revolutions/sec
        rpm=speed*60.0;

        return(rpm);
    }

    /////////////////////////////////////////////////////////////////
    //  Function:  double convert2Degrees(double counts)
    /////////////////////////////////////////////////////////////////
    //
    //  Purpose:
    //
    //  Arguments:
    //
    //  Returns:
    //
    //  Remarks:  One revolution of the motor shaft produces
    //            2048 counts.  The gear reduction of the 
    //            rotation system is 200:1.  This implies that
    //            360 degrees of rotation is 200*2048 counts
    //            or 8.78e-4 degrees per count.
    //
    /////////////////////////////////////////////////////////////////
    /////////////////////////////////////////////////////////////////
    public double convert2Degrees(double counts)
    {
        double degrees;

        degrees=8.78e-4*counts;
        return(degrees);
    }

    public double convert2Counts(double degrees)
    {
        double counts;

        counts=degrees/8.78e-4;
        return(counts);
    }
    
}      //  class definition

/////////////////////////////////////////////////////////////////
//  Function: 
/////////////////////////////////////////////////////////////////
//
//  Purpose:
//
//  Arguments:
//
//  Returns:
//
//  Remarks:
//
/////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////

    

