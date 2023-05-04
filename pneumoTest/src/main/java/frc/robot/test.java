package frc.robot;


/////////////////////////////////////////////////////////////////////
//  Class:  PneumoTestThread
/////////////////////////////////////////////////////////////////////
//
//Purpose: Implementation of a thread created within autonomous
//         that runs the pneumatics components on the test
//         board
//
//
//Remarks:  2/12/2022: Tested and works.
//
//
//
/////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////
class PneumoTestThread implements Runnable {

    //  Timing parame int  
    volatile int isactive;

    private long start_time=0;
    private long end_time=0;
    private double elapsed_time=0.0;

    private double duration=3000;  //  duration in msec
  	
	String name;
	Thread t;
	Runtime r = Runtime.getRuntime();
	private Delay delay;
	
	// Constructor
	PneumoTestThread(String threadname) {

		
		name = threadname;
		t = new Thread(this, name);
        
        System.out.printf("New thread: %s",name);
    
		delay = new Delay();

		//  Start the timer
		start_time=System.nanoTime();
        isactive=1;
     
		t.start(); // Start the thread
	}

	public void run() {

        int i;
		
       //  Two ways to go here - we could use an 'if' statement.  That would probably
       //  be safer in that we only intend to run this thread once.  Using a while()
       //  is dangerous in that we could be locked into an infinite loop if the
       //  thread fails to terminate.  'isactive' is set to '1' in the thread
       //  constructor and zeroed if we successfully interrupt the thread.
       while(isactive==1) {

            //  delay a couple of seconds to confirm that the
            //  compressor OFF command in robotInit() works as
            //  expected
            delay.delay_milliseconds(2000);

           //  Turn the compressor ON
            Robot.pneumo.compressorOn();

            //  Wait five seconds for pressure to build up
            delay.delay_milliseconds(5000);

            //  Cycle the cylinder ON/OFF five times
            //  with a half second between movements.
            for(i=0;i<5;i++)  {

                delay.delay_milliseconds(500);

                Robot.pneumo.enableDoubleSolenoidForward();

                delay.delay_milliseconds(500);

                Robot.pneumo.enableDoubleSolenoidReverse();

            }
    
            //  wait a few seconds before turning off the
            //  compressor
            delay.delay_milliseconds(duration);

            //  kill the compressor
            Robot.pneumo.compressorOff();
            
            //  interrupt (terminate)  the thread 
            t.interrupt();
                
                                
            // Wait for the thread to complete
            try {
                t.join();
            } catch (InterruptedException e) {
               
                System.out.printf("%s Interrupted",name);
                
                isactive=0; //  exit the while() loop
                
            }

            //  Output time to reach this point.  Should be stability
            //  time plus duration.
            end_time=System.nanoTime();
            elapsed_time=end_time-start_time;
            elapsed_time*=1e-6;  //  convert to milliseconds
            System.out.printf("Elapsed time = %.3 msec",elapsed_time);
           
	
        }  //  while(isactive==1)

        //  brute force release the memory
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
