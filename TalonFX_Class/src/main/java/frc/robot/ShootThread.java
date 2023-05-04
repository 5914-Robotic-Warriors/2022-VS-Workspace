/////////////////////////////////////////////////////////////////////
//  File:  ShootThread.java
/////////////////////////////////////////////////////////////////////
//
//Purpose:  Defines another thread class responsible for shooting the
//          ball.  Intended to be activated with an autonomous call.
//
//Programmers:  
//
//Revisions:  Initial Development 02/16/2021
//
//Remarks: 02/16/2021:  This thread version uses a while() 
//         loop based on t.isInterrupted().
//
//         Results:  This thread uses the repetitive application
//         of TalFX_Shooter function int setVelocity(double duration)
//         where duration is the motor duration in seconds after
//         stability is reached.  The use of thread.interrupt() is
//         successful, however the readback of the interrupt status
//         via thread.isInterrupted() is not.  I may not understand
//         the use of this function to the degree necessary although
//         documentation indicates it should return "true" if we
//         have interrupted the thread.  We clearly interrupt the
//         thread via entrance into the try/catch block.  Use is
//         made of a volatile int "isactive" to determine when to
//         exit the while() loop and it works as advertised.  Once
//         interrupted we enter a try/catch block where we change
//         the value of "isactive" to zero.
//
//         02/18/2021:  Working thread with dual motors.  Requires
//         bringing the duration out of the setVelocity().  This is 
//         good in that both autonomous and telep versions are now
//         the same.  Still have an issue with the isInterrupted
//         flag but this will be solved eventually.  We now have
//         a private data member "duration" that will specify
//         the number of msec we want the motors to run once
//         stability is established.
//         With regard to the interrupted flag, I now check for
//         isAlive() in the autonomous block of robot.java.
//         The thread does terminate as it should.  Eliminated
//         calls to functions isAlive() and isInterrupted() within
//         run();
//
//         02/19/2021:  Additional insight with regard to 
//         isInterrupted flag:  calling t.interrupt is
//         equivalent to calling CurrentThread().interrupt().
//         As such the call resets the flag to zero - as it
//         should, making our observations consistent with
//         what should happen.
//
/////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////
package frc.robot;

/////////////////////////////////////////////////////////////////////
//  Class:  ShootThread
/////////////////////////////////////////////////////////////////////
//
//Purpose: Implementation of a thread created within autonomous
//         that runs the shooter mechanism for a specific 
//         period of time following acheivement of target motor
//         velocity.  Can run more than one motor.
//
//
//Remarks:  2/20/21: robot_v1.6-alpha.zip
//
//          This thread works as advertised.  Expectation is that
//          it could be used in autonomous to shoot the ball at
//          the target.  Would need to be modified to include
//          ranging, computation, angle adjust, etc..
//
/////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////
class ShootThread implements Runnable {

    //  Timing parame int  
    volatile int isactive;
    volatile int lf_status;
    volatile int rf_status;

    private long start_time=0;
    private long end_time=0;
    private double elapsed_time=0.0;

    private double duration=3000;  //  duration in msec

  	
	String name;
	Thread t;
	Runtime r = Runtime.getRuntime();
	private Delay delay;
	
	// Constructor
	ShootThread(String threadname) {

		
		name = threadname;
		t = new Thread(this, name);
        //System.out.println("New thread: " + t); //  This is visible at the console.
        System.out.printf("New thread: %s",name);
    
		delay = new Delay();

		//  Start the timer
		start_time=System.nanoTime();
        isactive=1;
        lf_status=0;
        rf_status=0;
		t.start(); // Start the thread
	}

	public void run() {
		
       while(isactive==1) {

            //  start both motors, evaluate status of each with
            //  every pass through autonomous.
            if(lf_status!=1) {
                lf_status=Robot.lf_shoot.setVelocity_auto_();
            }
            delay.delay_milliseconds(10);
            if(rf_status!=1)  {	
                rf_status=Robot.rf_shoot.setVelocity_auto_();
            }	
            
            //  If both motors are stable, continue running for
            //  the duration, kill the motors and terminate the thread
			if((lf_status==1)&&(rf_status==1)) {
                
                //  keep the motors running for duration in msec
                delay.delay_milliseconds(duration);

                Robot.lf_shoot.stop();
                Robot.rf_shoot.stop();
                
                t.interrupt();
                
                                
                // Wait for the thread to complete
                try {
                    t.join();
                } catch (InterruptedException e) {
                    //System.out.println(name + " Interrupted.");
                    System.out.printf("%s Interrupted",name);
                    //delay.delay_milliseconds(15);
                    isactive=0; //  exit the while() loop
                  
                }

                //  Output time to reach this point.  Should be stability
                //  time plus duration.
                end_time=System.nanoTime();
                elapsed_time=end_time-start_time;
                elapsed_time*=1e-6;  //  convert to milliseconds
                System.out.printf("Elapsed time = %.3 msec",elapsed_time);
            }
	
        }  //  while(isactive==1)

        r.gc();
        return;
    }  //  run function


    //  Access functions for private data members
    int setDuration(double time_msec)
    {
        duration=time_msec;
        return(0);
    }

    double getDuration()
    {
        return(duration);
    }
}      //  class definition