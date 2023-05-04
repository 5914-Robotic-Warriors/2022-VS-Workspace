package frc.robot;

/////////////////////////////////////////////////////////////////////
//  Class:  AnalysisThread
/////////////////////////////////////////////////////////////////////
//
//Purpose: Implementation of a thread created within autonomous
//         that runs the shooter mechanism for a specific 
//         period of time following acheivement of target motor
//         velocity.  Can run more than one motor.  This specific
//         operates the motors a range in velocities, the goal
//         being to ultimately determine a set of starting
//         motor powers to quicken the convergence.
//
//
//Remarks:  Initial development 02/19/2021.
//
//        02/20/21:  robot_v1.6-alpha.zip
//        Thread working as advertized.  Should be modified in 
//        the future to create a fractional motor drive vs.
//        belt speed table.
//
//        02/23/2021:  Modified run to include velocities from
//        20ft/sec -> 100ft/sec.  Output records velocity and
//        fractional motor drive.  Will be useful in creating
//        a table for rapid convergence.  This will be part of
//        robot_v1.7-alpha.zip.
//
/////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////
class AnalysisThread implements Runnable {

    //  Status flags
    volatile int isactive;
    volatile int lf_status;
    volatile int rf_status;

    private double duration=2000;  //  duration in msec
 
    int speed_change;
    double v=30.0;
  	
	String name;
	Thread t;
	Runtime r = Runtime.getRuntime();
	private Delay delay;
	
	// Constructor
	AnalysisThread(String threadname) {
    	
		name = threadname;
		t = new Thread(this, name);
        System.out.println("New thread: " + t); //  This is visible at the console.
    
		delay = new Delay();

	    isactive=1;
        lf_status=0;
        rf_status=0;
        speed_change=1;
        v=0.0;

		t.start(); // Start the thread
	}

	public void run() {

       	//  primary run flag.
        while(isactive==1) {

                if(speed_change==1)  {

                    //  re-initialize motor count and init flag
                    //  these are private data members of TalFX_Shooter
                    Robot.lf_shoot.resetCount();
                    Robot.rf_shoot.resetCount();

                    Robot.lf_shoot.reInit();
                    Robot.rf_shoot.reInit();
                
                    //  reset the target belt velocity

                    Robot.lf_shoot.setTargetVelocity(v); 
                 
                    Robot.rf_shoot.setTargetVelocity(v); 
                
                    //  Compute rotational speed target in counts/100msec
                    Robot.lf_shoot.target_speed=Robot.lf_shoot.computeMotorSpeed(Robot.lf_shoot.getTargetVelocity());
                    Robot.rf_shoot.target_speed=Robot.rf_shoot.computeMotorSpeed(Robot.lf_shoot.getTargetVelocity());

                    //  zero the flag so we don't come back into this
                    //  block until we want to.
                    speed_change=0;
                }
                

                              
                if(rf_status!=1) {
                    rf_status=Robot.rf_shoot.setVelocity_auto();
                }
                delay.delay_milliseconds(10);
                if(lf_status!=1)  {	
                    lf_status=Robot.lf_shoot.setVelocity_auto();
                }
                
                //  If both motors are stable, continue running for
                //  the duration, reset flags and increment the
                //  velocity for the next speed change.
                if((lf_status==1)&&(rf_status==1)) {
                              
                    //  keep the motors running for duration in msec
                    delay.delay_milliseconds(duration);

                    //  reset flags
                    speed_change=1;
                    lf_status=0;
                    rf_status=0;

                    //  Increment the velocity
                    v+=5.0;
                   
                    //  put a limit on top speed
                    if(v>75.0)  {
                        isactive=0;
                        
                        //  Stop the motors
                        Robot.lf_shoot.stop();
                        Robot.rf_shoot.stop();

                    }
                   

                }
        
        }  //  while(isactive==1)    
                 

        //  Time to terminate the thread
        if(isactive==0)  {
            t.interrupt();
                            
            // Wait for the thread to complete
            try {
                t.join();
            } catch (InterruptedException e) {
                System.out.println(name + " Interrupted.");
                           
            }

            //  free memory resources
            r.gc();
            return;
        } // thread interrupt block
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
