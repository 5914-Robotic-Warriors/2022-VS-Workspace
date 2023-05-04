/////////////////////////////////////////////////////////////////////
//  File: trajectoryCompute.java
/////////////////////////////////////////////////////////////////////
//
//  Purpose:  This file contains functions that compute the angle
//            required to hit a target located at (x,y) from
//            location (x0,y0)  given the launch velocity.
//            
//            Note the following:
//
//            1.  The effect of air resistance is ignored.
//            2.  No compensation for the effect of spin.
//            3.  Classic trajectory physics.
//            4.  Units are in inches for physical dimensions
//                ft/sec for velocities.
//
//            The premise is that the projectile must have the
//            same time of light for the x travel and y travel.
//            When they are equal, the projectile hits the 
//            target.
//
//            Note that there are two possible solutions for
//            the y time-of-flight.  The function for the
//            y-dependence on time is a parabola.  One
//            solution will hit the y target prior to reaching
//            the azimuth, the other will hit the target
//            after reaching the azimuth.
//
//            For a variety of parameter values there will
//            not be a solution.
//
//  Environment:  Microsoft VS - java
//
//  Programmer:
//
//  Initial Development: 1/05/2020
//
//  Revisions:
//
//  1/10/2020:  Changed variables to "private" access.
//              Added access functions for the member variables.
//              This will prevent variable "naming" conflicts
//              with applications that use this class.
//
//  2/26/2021:  Changed distance parameters to feet and inches.
//              Factor in the change in x and y launch locations
//              with change in inclination angle.
//
//  8/12/2021:  Changed coordinate system to have the origin
//              (0,0) at the pivot of the launch arm.  Added
//              the variable y_floor which is the y distance
//              from the origin to the floor.  Eliminated
//              x_0 and y_0.  This simplifies visualization
//              and computation of values.  Several problems
//              were encountered and corrected.  Also eliminated
//              "static" declaration of private member variables.
//
//  10/13/2021: Changed from unformatted "println(...)" to a formatted
//              printf(...)            
//
/////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////

package frc.robot;

class   TrajectoryComputations {

    //  Private member variables
    
    //  x_target:  This is the measured distance from the origin
    //             to the target wall;      
    private double x_target;

    //  x:  This is the x_distance to target from the ball
    //      center to the target and includes the effect of
    //      inclination angle.  It will increase as the
    //      inclination angle increases.  It is given by
    //
    //      x = x_target - arm_length*cos(theta))
    private double x;    

    // y_target:  This is the target height in inches from the floor.
    private double y_target;    //  target y position in inches

    //  y:  This is the target y position in inches (includes inclination)
    //      this represents the y difference from the ball position to the
    //      target position.  It will decrease as the inclination angle
    //      increases.
    //
    //      y = y_target - y_floor - arm_length*sin(theta)
    private double y;  
   
    //  These are fixed parameters based on robot contruction
    static double y_floor = 24.0;
    static double arm_length=35.0;

    //  Don't forget our friend accleration due to gravity
    final static double g = 32.0;

    double v;    //  firing velocity in ft/second
    double theta;  //  firing angle in degrees


   //  Default constructor
   TrajectoryComputations()
   {      
       x_target =0; //  Will be specified - horizontal distance form origin to target.
       y_target = 98.25;  //  This is fixed per game rules
       x=0.0;       //  This is computed based on inclination angle
       y=0.0;       //  This is computed based on inclination angle
       v=0.0;       //  Specified
       theta=0.0;   //  Inclination angle, computed or specified depending on function
   }

      //  Overloaded constructor
      //  Note:  we could include a computation of the line-of-sight inclination angle within this
      //  constructor?
    TrajectoryComputations(double targ_x,double launch_v,double targ_y,double launch_angle)
    {
        x_target=targ_x;
        y_target=targ_y;
        v=launch_v;
        theta=launch_angle;
    }
 
    void set_x_target(double targ_x)  
    {
         x_target=targ_x;

    }
    double get_x_target()  
    {
         return(x_target);

    }

    void set_y_target(double targ_y)  
    {
         y_target=targ_y;

    }   
    double get_y_target()  
    {
         return(y_target);

    }
   
    double get_x()  
    {
         return(x);

    }
    double get_y()  
    {
         return(y);

    }

    void set_y_floor(double yfloor)  
    {
         y_floor = yfloor;
    }

    double get_y_floor()  
    {
         return(y_floor);

    }

    void set_arm_length(double armlength)  
    {
         arm_length = armlength;
    }

    double get_arm_length()  
    {
         return(arm_length);  

    }

    
    /////////////////////////////////////////////////////////////////
    //  Function:     double compute_tof_x( ... )
    /////////////////////////////////////////////////////////////////
    //
    //  Purpose:  Given the velocity and launch angle, this function
    //            computes the time of flight along the horizontal.
    //
    //  Arguments: Launch velocity in ft/sec, and launch angle in degrees.  
    //
    //  Returns:  The time of flight in seconds as double
    //
    //  Remarks:  Angles are expressed in degrees.
    //            Note that distances are in inches and velocity
    //            must be converted from ft/sec to inches/second.
    //            It is assumed that the 'x' target distance and arm
    //            length are known.
    //
    /////////////////////////////////////////////////////////////////
    /////////////////////////////////////////////////////////////////
    double compute_tof_x(double v,double theta_deg)
    {
        double tof;
        double v0_x;
        double theta_r;

     
        //  Convert to radians
        theta_r=theta_deg*Math.PI/180.0;

        //  compensate x for inclination angle
        x = x_target-arm_length*Math.cos(theta_r);
        
        //  Compute horizontal velocity component
        v0_x=v*Math.cos(theta_r);

        //  Convert to inches per second
        v0_x*=12.0;

        //  Compute time of flight
        tof=x/v0_x;
        return(tof);

    }

    /////////////////////////////////////////////////////////////////
    //  Function:     double compute_tof_y( ... )
    /////////////////////////////////////////////////////////////////
    //
    //  Purpose:  Given the velocity and distances, this function
    //            computes the time of flight along the vertical.
    //            This function computes the shortest time, the
    //            arrival prior to the azimuth.
    //
    //  Arguments:  Launch velocity, and launch angle.  Velocity
    //             is in ft/sec, angle in degrees.
    //
    //  Returns:  The time of flight in seconds as double
    //
    //  Remarks:  
    //
    /////////////////////////////////////////////////////////////////
    /////////////////////////////////////////////////////////////////
    double compute_tof_y_min(double v,double theta_deg)
    {
        double tof;
        double v0_y;
        double theta_r;
        double temp_arg;

        //  Convert to radians
        theta_r=theta_deg*Math.PI/180.0;

        //  Compensate y for inclination angle.  It
        //  gets smaller with increasing inclination angle.
        y=y_target-y_floor-(arm_length*Math.sin(theta_r));

        //  Compute vertical component
        v0_y=v*Math.sin(theta_r);

        //  convert to inches/second
        v0_y*=12.0;

        //  Compute the argument of the square root
        temp_arg=(v0_y*v0_y) - 4*(g/2.0)*(y);

   
        //  Test for real values
        if(temp_arg<0.0)  {
            return(-999.99);
        }

        //  Compute time of flight for y using the
        //  negative sign of the quadratic formula.
        tof=(v0_y - Math.sqrt(temp_arg))/g;
        return(tof);

    }

    /////////////////////////////////////////////////////////////////
    //  Function:     double compute_tof_y_max( ... )
    /////////////////////////////////////////////////////////////////
    //
    //  Purpose:  Given the velocity and distances, this function
    //            computes the time of flight along the vertical.
    //            This function computes the longer time, the
    //            arrival after the azimuth.
    //
    //  Arguments:Launch velocity in ft/second, and launch angle
    //            in degrees.  
    //
    //  Returns:  The time of flight in seconds as double
    //
    //  Remarks:  Distances in inches, angles are expressed in degrees.
    //
    /////////////////////////////////////////////////////////////////
    /////////////////////////////////////////////////////////////////
    double compute_tof_y_max(double v,double theta_deg)
    {
        double tof;
        double v0_y;
        double theta_r;
        double temp_arg;

        //  Convert to radians
        theta_r=theta_deg*Math.PI/180.0;

         //  Compensate y for inclination angle
         y=y_target-y_floor-(arm_length*Math.sin(theta_r));


        //  Compute vertical component
        v0_y=v*Math.sin(theta_r);

        //  Convert to inches/second
        v0_y*=12.0;

       //  Compute the argument of the square root
       temp_arg=(v0_y*v0_y) - 4*(g/2.0)*(y);

       //  Test for real values
       if(temp_arg<0.0)  {
           return(-999.99);
       }

       //  Compute time of flight for y
       tof=(v0_y + Math.sqrt(temp_arg))/g;
       return(tof);

    }


    /////////////////////////////////////////////////////////////////
    //  Function:  computeMinTheta(double x)
    /////////////////////////////////////////////////////////////////
    //
    //  Purpose:  Computes the line-of-sight angle from the 
    //            arm pivot to the center of the target.
    //
    //  Arguments:double x representing the horizontal distance
    //            from the ball center at zero degrees inclination
    //            to the target.
    //
    //  Returns:  The launch angle in radians expressed as double.
    //
    //  Remarks:  This function computes the line of sight from
    //            the pivot arm to the center of the target.  
    //
    /////////////////////////////////////////////////////////////////
    /////////////////////////////////////////////////////////////////
    double computeMinTheta(double x_target)
    {

        double temp;
        double min_theta;

        temp=(y_target-y_floor)/x_target;

        min_theta=Math.atan(temp);

        return(min_theta);

    }

    /////////////////////////////////////////////////////////////////
    //  Function:  solve4Theta()
    /////////////////////////////////////////////////////////////////
    //
    //  Purpose:  Computes the launch angle in degrees where the x and
    //            y time-of-flight values are equal.  It firsts 
    //            determines the minimum launch angle represented by
    //            a direct line of sight.  From this point the
    //            routine searchs for the minimum difference between
    //            the x and y tofs.  If the routine fails to minimize
    //            the difference to less than one tenth of a second it is 
    //            assumed that a solution does not exist.
    //
    //  Arguments:void
    //
    //  Returns:  The launch angle in degress expressed as double.
    //
    //  Remarks:  This function computes the shortest route, i.e.,
    //            y target hit before azimuth.
    //
    /////////////////////////////////////////////////////////////////
    /////////////////////////////////////////////////////////////////
    double solve4Theta()
    {
        int debug=1;

        double min_theta;
        double theta;
        double best_theta=0.0;
        double delta;
        double min_delta=2000.00;
        double launch_theta;
        double tof_x=0.0;
        double tof_y=0.0;
        double best_tof_x=0;
        double best_tof_y=0;
   


        //  minimum theta is direct line of sight
        //  this function will return the value in radians
        min_theta=computeMinTheta(x_target);

        //  convert to degrees
        min_theta=(180.0/Math.PI)*min_theta;

        if(debug==1)  {
            System.out.printf("Line of Sight Angle = %.3f deg\n",min_theta);
        }

        //  Search for minimum delta of tof x & y
        for(theta=min_theta;theta<90.0;theta+=.01)  {
            //  These functions require that theta is in degrees.
            tof_y=compute_tof_y_min(v,theta);  
            tof_x=compute_tof_x(v,theta);  

            delta=Math.abs(tof_y-tof_x);
            if(delta<min_delta)  {
                min_delta=delta;
                best_theta=theta;
                best_tof_x=tof_x;
                best_tof_y=tof_y;
            }

        }
        

        //  Limit tof difference to 0.1 second
        if(min_delta>0.1)  {
            System.out.println("No solution, theta min: TOF difference > 0.1 seconds.");
            return(0.0);
        }

        if(debug==1)  {
            System.out.printf("delta tmin = %.4f  best_theta = %.4f\n",min_delta,best_theta);
            System.out.printf("TOF_y = %.3f\n",best_tof_y);
            System.out.printf("TOF_x = %.3f\n",best_tof_x);
        }

        launch_theta=best_theta;
        return(launch_theta);
    }

    /////////////////////////////////////////////////////////////////
    //  Function:  solve4Theta_max()
    /////////////////////////////////////////////////////////////////
    //
    //  Purpose:  Computes the launch angle in degrees where the x and
    //            y time-of-flight values are equal.  It firsts 
    //            determines the minimum launch angle represented by
    //            a direct line of sight.  From this point the
    //            routine searchs for the minimum difference between
    //            the x and y tofs.  If the routine fails to minimize
    //            the difference to less than one tenth of asecond it is 
    //            assumed that a solution does not exist.
    //
    //  Arguments:void
    //
    //  Returns:  The launch angle in degress expressed as double.
    //
    //  Remarks:  This function computes the longest route, i.e.,
    //            y target hit after azimuth.
    //
    /////////////////////////////////////////////////////////////////
    /////////////////////////////////////////////////////////////////
    double solve4Theta_max()
    {
        int debug=1;

        double min_theta;
        double theta;
        double best_theta=0.0;
        double delta;
        double min_delta=2000.00;
        double launch_theta;
        double tof_x=0.0;
        double tof_y=0.0;
        double best_tof_x=0;
        double best_tof_y=0;

        //  minimum theta is direct line of sight
        //  this function will return the value in radians
        min_theta=computeMinTheta(x_target);

        //  convert to degrees
        min_theta=(180.0/Math.PI)*min_theta;

        //  Search for minimum delta of tof x & y
        for(theta=min_theta;theta<90.0;theta+=.01)  {
            //  These functions require that theta is in degrees.
            tof_y=compute_tof_y_max(v,theta);  
            tof_x=compute_tof_x(v,theta);  

            delta=Math.abs(tof_y-tof_x);
            if(delta<min_delta)  {
                min_delta=delta;
                best_theta=theta;
                best_tof_x=tof_x;
                best_tof_y=tof_y;
            }

        }

        //  Limit tof difference to 0.1 second
        if(min_delta>0.1)  {
            System.out.println("No solution, theta max: TOF difference > 0.1 seconds.");
            return(0.0);
        }

        if(debug==1)  {
            System.out.printf("delta tmax = %.4f  best_theta = %.4f\n",min_delta,best_theta);
            System.out.printf("TOF_y = %.3f\n",best_tof_y);
            System.out.printf("TOF_x = %.3f\n",best_tof_x);
        }
       
        launch_theta=best_theta;
        return(launch_theta);
    }

    /////////////////////////////////////////////////////////////////
    //  Function:  double computeYposition(double theta,double v,double t)
    /////////////////////////////////////////////////////////////////
    //
    //  Purpose:  Computes the Y position as a function of the
    //            inclination angle and time.
    //
    //  Arguments: double theta  (inclination angle)
    //             double v, velocity in (ft/sec)
    //             double t, time  (in seconds)
    //
    //  Returns:  double representing the Y position in inches
    //
    //  Remarks:  
    //
    /////////////////////////////////////////////////////////////////
    /////////////////////////////////////////////////////////////////
    double computeYposition(double theta,double v,double t)
    {
        double ypos;
        double v_y;
        double init_y;
    
        //  Convert to radians
        theta = theta*Math.PI/180.0;

        //  compensate y for inclination angle
        init_y = arm_length*(Math.sin(theta));

        //  Convert velocity to inches per second
        v*=12.0;

        //  Compute vertical velocity component
        v_y=v*Math.sin(theta);

    
        ypos=init_y + v_y*t - 0.5*g*t*t;
    
        return(ypos);
    }


    /////////////////////////////////////////////////////////////////
    //  Function:  double computeXposition(double theta,
    //                               double velocity,double t)
    /////////////////////////////////////////////////////////////////
    //
    //  Purpose:  Computes the X position as a function of the
    //            inclination angle and time.
    //
    //  Arguments: double theta  (inclination angle in degrees)
    //             double v, velocity int (ft/sec)
    //             double t, time  (in seconds)
    //
    //  Returns:  double representing the X position in inches  
    //
    //  Remarks:  
    //
    /////////////////////////////////////////////////////////////////
    /////////////////////////////////////////////////////////////////
    double computeXposition(double theta,double v,double t)
    {
        double init_x;
        double xpos;
        double v_x;


        //  Convert to radians
        theta=theta*Math.PI/180.0;

        //  compensate x for inclination angle
        init_x=arm_length*Math.cos(theta);

        //  Convert velocity to inches per second
        v*=12.0;

        //  Compute horizontal velocity component
        v_x=v*Math.cos(theta);

    
        xpos=init_x + v_x*t;
        

        return(xpos);


    }





    //  Function template
    /////////////////////////////////////////////////////////////////
    //  Function:  
    /////////////////////////////////////////////////////////////////
    //
    //  Purpose:  
    //
    //  Arguments:void
    //
    //  Returns:  
    //
    //  Remarks:  
    //
    /////////////////////////////////////////////////////////////////
    /////////////////////////////////////////////////////////////////


}