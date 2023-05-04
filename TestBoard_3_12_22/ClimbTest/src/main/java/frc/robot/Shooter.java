/////////////////////////////////////////////////////////////////////
//  File:  Shooter.java
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
//            The Shooter uses a NEO motor and no pneumatics
//            Setting it at a fixed speed is important.  The
//            actual shooter on the Robot uses three NEO motors
//            We have a single on the test board.
//
//  Inception:  3/12/2022
//
//
//////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////

package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxRelativeEncoder;
import com.revrobotics.RelativeEncoder;

public class Shooter extends Robot {


    CANSparkMax neo;
    RelativeEncoder neos_encoder;

    Delay delay;

    double neo_position;
    double neo_velocity;

    double neo_power=0.5;  //  starting power

    private final double NEO_deadband=50;

    int neo_init=1;

    //  constructor
    Shooter()
    {
       neo = new CANSparkMax(2, MotorType.kBrushless);
       neos_encoder= neo.getEncoder(SparkMaxRelativeEncoder.Type.kHallSensor,42);
       delay=new Delay();
    }

    /////////////////////////////////////////////////////////////////
    //  Function:  double setNEOspeed(double target)
    /////////////////////////////////////////////////////////////////
    //
    //  Purpose:  Sets the speed of the NEO motor in the default
    //            units of RPM
    //
    //  Arguments:Accepts the target RPM as double
    //
    //  Returns:  A double representing the error
    //
    //  Remarks:  The NEO encoder returns RPM when read for velocity.  
    //
    //  3/8/22:  Tested appears to work.  Refinement would be
    //           to adjust the power increment depending on the
    //           size of the calculated error.
    //  3/9/22:  The 20msec delay between commanding power and
    //           reading result is very important.  10msec was
    //           not adequate.  Proportional tree appears to
    //           result in stable operation.
    //
    /////////////////////////////////////////////////////////////////
    /////////////////////////////////////////////////////////////////
    public double setNEOspeed(double target)
    {
        int debug=0;  //  set to '1' if screen printouts are wanted.
        double error;
        double speed;
       
        if(neo_init==1)  {
          if(target<0.0)neo_power*=-1.0;
          neo.set(neo_power);
          neo_init=0;
        }
        delay.delay_milliseconds(20.0);
        speed=neos_encoder.getVelocity();
        error=target-speed;

        if(Math.abs(error)>NEO_deadband)  {
            error=target-speed;
            if(error>0.0)  {
              if(error>300.0) {
                neo_power+=0.01;
              }  else if((error>100.0)&&(error<300.0)) {
                neo_power+=0.005;
              }  else if((error>NEO_deadband)&&(error<100.0)) {
                neo_power+=0.001;
              }  else {
                ;
              }
           
            } else if(error<0.0)  {
              if(error<-300.0) {
                neo_power-=0.01;
              }  else if((error<-100.0)&&(error>-300.0)) {
                neo_power-=0.005;
              }  else if((error<-NEO_deadband)&&(error>-100.0)) {
                neo_power-=0.001;
              }  else {
                ;
              }
            }
            neo.set(neo_power);
            delay.delay_milliseconds(20.0);
            speed=neos_encoder.getVelocity();
        }  else  {
          neo.set(neo_power);
          delay.delay_milliseconds(20.0);
          speed=neos_encoder.getVelocity();
        }
        if(debug==1) {
          System.out.printf("NEO speed = %.3f RPM  ",speed);
          System.out.printf("NEO power = %.3f\n",neo_power);
        }
        return(error);
    }

    public int setNEOoff()
    {
      neo.set(0.0);
      return(0);
    }

    
}
