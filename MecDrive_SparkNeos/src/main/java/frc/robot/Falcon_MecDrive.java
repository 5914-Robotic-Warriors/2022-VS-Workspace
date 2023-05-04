/////////////////////////////////////////////////////////////////////
//  File:  Falcon_MecDrive.java
/////////////////////////////////////////////////////////////////////
//
//  Purpose:  The intent is to create a class entirely devoted to
//            a robot mecanum drive using the FalconFX brushless
//            motors.  The only external dependence is on a "Constants" class for
//            motor CAN ID's.
//
//  Compile Environment:  Java via Microsoft VS
//
//  Inception Date:  Originally developed as part of the 
//            2020-2021 robot as robotDrive.java.
//            Modified for FalconFX 2/19/2022.
//
//  Revisions: The only required input from the Constants class are
//            the CAN ID's for the four motors.  
//
//            1/10/2022:  Imported into new VS/WPILIB for 2022 FRC
//            Changes required for deprecated functions.  Compiled
//            successfully.
//            02/19/2022 Changes:
//            *  FalconFX brushless motor
//            *  FalconFX integrated encoder
//            *  new wpilib and revrobotics libraries for 2022
//            *  Use of Constants class for fixed parameters
//
//            Many of the functions contained with in this class are
//            used for autonomous operation.
//            The autonomous functions are not to be used for
//            driving operations in that they contain while()
//            loops that will defeat the watchdog nature of the
//            timed robot application.
//
/////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////
package frc.robot;


import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.drive.MecanumDrive;


class Falcon_MecDrive extends Robot {

    // Fixed parameters for driveFwd(...)/driveBwd(...)
    // It is expected that these may be fine-tuned
    private final double START_SPEED = 0.1; // Also used in acceleration functions.
    private final double MAX_SPEED = 0.6;
    private final double MIN_SPEED = 0.1;
    private final double BRAKE_SPEED = 0.3;
    private final double BRAKE_FRACTION = 0.4;

    // Fixed parameters for console updates and while() loop escapes.
    //private final int ENC_CONSOLE_UPDATE = 20;
    //private final int ENC_LOOP_ESCAPE = 250; 
    private final int GYRO_CONSOLE_UPDATE = 20;
    private final int GYRO_LOOP_ESCAPE = 200;

    // Fixed parameters for gyro operation. Specified here to facilitate
    // changes without confusion in the various functions using these
    // variables.
    private final double ROT_SPEED = 0.5; // Starting rotation speed for turning
    // As we approach the target we reduce the speed by this factor
    private final double ROT_ATTEN = 1.5;
    // proximity (in degrees) to target angle stage 1 rotation attenuation rate
    private final double ANGL_PROX_1 = 25.0;
    // proximity (in degrees) to target angle stage 2 rotation attenuation rate
    private final double ANGL_PROX_2 = 5.0;


     /*                      Drive Encoder Parameters: updated 2/19/2022
     * Drive encoders are on the FalconFX drive motors. The output from the encoders is 
     * 2048 counts per shaft revolution.  A gear reduction of 12.75:1 would imply 12.75  
     * times 2048 per revolution of the output shaft (26112 counts per output shaft
     * revolution).
     * An 8 inch wheel diameter would imply a distance traveled of PI*8.0 = 25.13 inches. 
     * Distance precision of the encoder/gearbox combination is 25.13/26112 = 9.62e-4 
     * inches per count. Specification of gear reduction and wheel diameter as well 
     * as the CAN addresses of the drive motors is in the Constants class.
     *  
     * Encoder position is read from a CANEncoder object by calling the
     * frontLeft.getSelectedSensorPosition();  method.
     * 
     * At present only one encoder is used.  This can be changed if required
     */
    
    private double encoder_precision; // inches per encoder count

  
    // Initial drive encoder position.
    public double initDrivePosition;

    // Initial drive gyro angle.
    public double initDriveGyroAngle;

    // Listing the drive motors.
    WPI_TalonFX frontLeft;
    WPI_TalonFX frontRight;
    WPI_TalonFX backLeft;
    WPI_TalonFX backRight;

    double frontLeftEncoderPosition;

    MecanumDrive mecanumDrive;
    private Delay delay;

    ADXRS450_Gyro driveGyro;

    Constants constants;
    
    //  Constructor for MecDrive()
    Falcon_MecDrive()  {

        delay=new Delay();

        constants = new Constants();

        // Creating the MecanumDrive constructor, which links all 4 motors together.
        
        // Constructing the motors, giving them their IDs, and making them brushless.
        frontLeft = new WPI_TalonFX(constants.frontLeftDrive);
        frontRight = new WPI_TalonFX(constants.frontRightDrive);
        backLeft = new WPI_TalonFX(constants.backLeftDrive);
        backRight = new WPI_TalonFX(constants.backRightDrive);

         // Invert the right side motors because we need it to work with the magic function
        frontRight.setInverted(true);
        backRight.setInverted(true);

    
        mecanumDrive = new MecanumDrive(frontLeft, backLeft, frontRight, backRight);


        // Encoder for the front left drive motor.
        frontLeft.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor,0,50);

        frontLeft.setSelectedSensorPosition(0.0);
        frontLeftEncoderPosition=frontLeft.getSelectedSensorPosition();
        
        // Gyro for auto drive functions.
        driveGyro = new ADXRS450_Gyro(); 

        // Set the drive motors to coast mode to help prevent tipping,
        // and to make the drive less jerky.
    

        // We want to establish an initial encoder reading. This will enable resetting
        // encoder position to zero when we start moving. We use absolute values to
        // make the subsequent subtraction more easily interpreted.
        frontLeft.setSelectedSensorPosition(0.0);
        frontLeft.getSelectedSensorPosition(); // should be zero

        //  Setup the encoder resolution with the wheel diameter and gear reduction
        encoder_precision=computeEncoderPrecision(constants.wheel_diameter,constants.gear_reduction);

        //  Setup the initial heading
        initDriveGyroAngle = driveGyro.getAngle();
        
    
    }

   

    //  This returns the precision of the system in inches per count
    //  We have 2048 counts per motor shaft revolution
    double computeEncoderPrecision(double diameter,double reduction)
    {
        double enc_precision;
        double counts_per_outputshaft;
        double inches_per_outputshaft;

        //  Counts for 1 revolution of the output shaft
        counts_per_outputshaft=2048.0*reduction;

        //  Inches of travel for one revolution of the output shaft
        inches_per_outputshaft=Math.PI*diameter;
    
        //  Compute inches per count
        enc_precision=inches_per_outputshaft/counts_per_outputshaft;

        return(enc_precision);
    }


    /////////////////////////////////////////////////////////////////////
    // Function: strafeLeft(...)
    /////////////////////////////////////////////////////////////////////
    //
    // Purpose: Used for strafing left, without encoder magic.
    //
    // Arguments: double strafeSpeed
    //
    // Returns: void
    //
    // Remarks: Created on 3/07/2020.
    //          12/12/2021:  Need to check sign of movements.
    //
    /////////////////////////////////////////////////////////////////////
    /////////////////////////////////////////////////////////////////////
    public void strafeLeft(double strafeSpeed) {

        // Strafe left at the input speed.
        frontLeft.set(-strafeSpeed);
        backLeft.set(-strafeSpeed);
        frontRight.set(strafeSpeed);
        backRight.set(strafeSpeed);
    }

    /////////////////////////////////////////////////////////////////////
    // Function: strafeRight(...)
    /////////////////////////////////////////////////////////////////////
    //
    // Purpose: Used for strafing right, without encoder magic.
    //
    // Arguments: double strafeSpeed
    //
    // Returns: void
    //
    // Remarks: Created on 3/07/2020. Created on 3/07/2020.
    //          12/12/2021:  Need to check sign of movements.
    //
    /////////////////////////////////////////////////////////////////////
    /////////////////////////////////////////////////////////////////////
    public void strafeRight(double strafeSpeed) {

        // Strafe right at the inputted speed.
        frontLeft.set(strafeSpeed);
        backLeft.set(strafeSpeed);
        frontRight.set(-strafeSpeed);
        backRight.set(-strafeSpeed);
    }


    /////////////////////////////////////////////////////////////////////
    // Function: strafeLeftAuto(...)
    /////////////////////////////////////////////////////////////////////
    //
    // Purpose: Used for strafing left in autonomous.
    //
    // Arguments: double speed, double time (how long to strafe in seconds).
    //
    // Returns: void
    //
    // Remarks:
    //
    /////////////////////////////////////////////////////////////////////
    /////////////////////////////////////////////////////////////////////
    public void strafeLeftAuto(double speed, double time) {

        // Strafe at the inputted speed.
        frontLeft.set(-speed);
        backLeft.set(-speed);
        frontRight.set(speed);
        backRight.set(speed);

        // Run the motors and stop after these many seconds.
        delay.delay_seconds(time);

        // Stop the motors after "time" amount of seconds.
        frontLeft.set(0);
        backLeft.set(0);
        frontRight.set(0);
        backRight.set(0);

    }

    /////////////////////////////////////////////////////////////////////
    // Function: strafeRightAuto(...)
    /////////////////////////////////////////////////////////////////////
    //
    // Purpose: Used for strafing right in autonomous.
    //
    // Arguments: double speed, double time (how long to strafe in seconds).
    //
    // Returns: void
    //
    // Remarks:
    //
    /////////////////////////////////////////////////////////////////////
    /////////////////////////////////////////////////////////////////////
    public void strafeRightAuto(double speed, double time) {

        // Strafe at the inputted speed.
        frontLeft.set(speed);
        backLeft.set(speed);
        frontRight.set(-speed);
        backRight.set(-speed);

        // Run the motors and stop after these many seconds.
        delay.delay_seconds(time);

        // Stop the motors after "time" amount of seconds.
        frontLeft.set(0);
        backLeft.set(0);
        frontRight.set(0);
        backRight.set(0);
    }

    /////////////////////////////////////////////////////////////////////
    // Function: public int driveFwd(double distance)
    /////////////////////////////////////////////////////////////////////
    //
    // Purpose: Drives the robot forward the distance in feet specified
    // by the argument.
    //
    // Arguments:double distance, The distance to be traveled in feet.
    //
    // Returns: A double representing overshoot/undershoot of the movement
    // in inches.
    //
    // Remarks: 02/09/2020: Modified to include acceleration/deceleration
    // 02/11/2020: Noted that the inequality regarding fraction
    // should have been '>' vs. '<'
    // Increased delays within while() loops
    // Reduced print statements to make it easier
    // to determine position and fraction.
    //
    /////////////////////////////////////////////////////////////////////
    /////////////////////////////////////////////////////////////////////
    public double driveFwd(double distance) {
        double counts; // Encoder counts.
        double initial_position; // Our initial position.
        double current_position; // CUrrent position of the robot.
        double target; // Init position + encoder counts.

        double fraction; // How close we are to our target.
        double heading; // Initial gyro angle.

        double error; // Overshoot/undershoot.

        // Determine where we are pointing - we want to maintain this heading during
        // this forward movement.
        heading = driveGyro.getAngle();
        frontLeft.setSelectedSensorPosition(0.0);
        frontLeft.getSelectedSensorPosition(); // should be zero
        // Read encoder #1, get the initial encoder position and assign it to
        // the current position. Calculate the number of encoder counts
        // necessary to reach the final destination (target).
        initial_position = frontLeft.getSelectedSensorPosition(); // should be zero 

        System.out.println("initPos = " + initial_position);

        current_position = initial_position;
        counts = calcCounts_SAE(distance);
        target = initial_position + counts;

        // fraction starts out as equal to 1.0 and decreases as we approach the target.
        // fraction is counts remaining divided by total counts.
        fraction = Math.abs((target - current_position) / (target - initial_position));

        System.out
                .println("initial_position = " + initial_position + " target = " + target + " fraction = " + fraction);

        // We attempt a bit of proportional control as we approach the target. We want
        // to slow down so that we don't overshoot. These fractions appear to work.
        // We drive at high speed for 80% of the distance and then slow. On carpet this
        // seemed to work very well for distance of 10 feet.
        // We want braking enabled.
        // We also need to put a timer within the while() loop to provide an escape in
        // the event that the system gets lost during autonomous requiring a restart of
        // the program.

        // Get moving and accelerate to MAX_SPEED.
        if (current_position < target) {
            accelerateFwd();
        }
        System.out.println("Acceleration complete");

        // Monitor position and continue to travel at max speed until
        // fraction<BRAKE_FRACTION. Note that fraction starts out at 1.0
        // and decreases as we approach the target encoder value.
        // 02/11/2020: Changed the sign of the inequality to '>'
        // Changed the delay from 0.01 to 0.02
        // Moved the position update outside of the while() loop
        while ((current_position < target) && (fraction > BRAKE_FRACTION)) {
            moveFwd(MAX_SPEED, heading);
            current_position = frontLeft.getSelectedSensorPosition(); 
            fraction = Math.abs((target - current_position) / (target - initial_position));
            delay.delay_seconds(0.02);
        }

        // Where are we?
        System.out
                .println("current_position = " + current_position + " target = " + target + " fraction = " + fraction);
        System.out.println("Decelerating");
        // Ok, we should be at the braking fraction. Time to decelerate to BRAKE_SPEED.
        decelerateFwd();

        // Continue at BRAKE_SPEED until we reach the target encoder value
        // 02/11/2020: Changed the delay from 0.01 to 0.02
        // 02/11/2020: Moved the position update outside of the while() loop
        while (current_position < target) {
            moveFwd(BRAKE_SPEED, heading);
            delay.delay_seconds(0.02);
            current_position = frontLeft.getSelectedSensorPosition(); 
        }

        System.out
                .println("current_position = " + current_position + " target = " + target + " fraction = " + fraction);

        // stop movement, compute error (overshoot or undershoot)
        mecanumDrive.driveCartesian(0, 0, 0);
        current_position = frontLeft.getSelectedSensorPosition(); // should be zero
        error = calcDistance_SAE(current_position - target);
        System.out.println("error = " + error + " inches");

        return (error);
    }

    ///////////////////////////////////////////////////////////////////////////
    // Function: void moveFwd(double speed,double target)
    ///////////////////////////////////////////////////////////////////////////
    //
    // Purpose: Uses on_board gyro to drive the robot straight forward
    // given a target angle as an argument.
    // Requires use of the arcadeDrive function with the first
    // argument being the forward speed and the second being
    // the turn applied.
    //
    // Arguments: double speed. Must be between -1.0 and 1.0.
    // double heading - the target angle.
    //
    // Returns: void
    //
    // Remarks: This function will be called every 20 msec. Updating
    // the position and heading each time it is called will
    // overwhelm the system.
    // 02/11/2020: Commented out the print statement.
    //
    ////////////////////////////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////////////
    public void moveFwd(double speed, double heading) {

        double corr = 0.2;
        double angle = 0;
        double delta; // The difference between the target and measured angle

        angle = driveGyro.getAngle();

        delta = angle - heading;

        // According to the documentation for DifferentialDrive.arcadeDrive(speed,turn)
        // the arguments are squared to accommodate lower drive speeds. If for example
        // the gain coefficient is 0.05 and the angle error is 5 degrees, the turning
        // argument would be 0.25*0.25 = 0.0625. This is a pretty slow correction.
        // We needed a larger correction factor - trying 0.2 for now. The range for
        // the turn is -1.0 to 1.0. Positive values are said to turn clockwise,
        // negatives counterclockwise.
        mecanumDrive.driveCartesian(0, speed, -corr * delta);

        // System.out.println(" heading = " + heading + " angle = " + angle + " delta =
        // " + delta);

    }

    /////////////////////////////////////////////////////////////////////
    // Function: accelerateFwd( ... )
    /////////////////////////////////////////////////////////////////////
    //
    // Purpose: Used for accelerating the robot with driveFwd().
    //
    // Arguments: double heading, i.e., the direction of travel.
    //
    // Returns: void
    //
    // Remarks: The listed speed increment and delay will allow
    // approximately 2 seconds to accelerate from a
    // speed of 0.1 to 0.6. We may want to alter these
    // variables depending on the distance.
    //
    // 02/11/2020: Simplified the acceleration phase of
    // the movement. Eliminated reading of angle and
    // direction correction. This allowed eliminating
    // the "heading" argument.
    //
    /////////////////////////////////////////////////////////////////////
    /////////////////////////////////////////////////////////////////////
    void accelerateFwd() {
        double speed = 0.1;

        while (speed < MAX_SPEED) {
            mecanumDrive.driveCartesian(0, speed, 0);
            delay.delay_seconds(0.02);
            speed += 0.01;
        }
    }

    /////////////////////////////////////////////////////////////////////
    // Function: decelerateFwd( ... )
    /////////////////////////////////////////////////////////////////////
    //
    // Purpose: Used for decelerating the robot with driveFwd().
    //
    // Arguments: void
    //
    // Returns: void
    //
    // Remarks: The listed speed increment and delay will allow
    // approximately 2 seconds to decelerate from a
    // speed of 0.1 to 0.6. We may want to alter these
    // variables depending on the distance.
    //
    // 02/11/2020: Simplified the deceleration phase of
    // the movement. Eliminated reading of angle and
    // direction correction.
    //
    /////////////////////////////////////////////////////////////////////
    /////////////////////////////////////////////////////////////////////
    void decelerateFwd() {
        double speed = MAX_SPEED;

        while (speed > MIN_SPEED) {
            mecanumDrive.driveCartesian(0, speed, 0);
            delay.delay_seconds(0.02);
            speed -= 0.01;
        }
    }

    /////////////////////////////////////////////////////////////////////
    // Function: public int driveBwd(double distance)
    /////////////////////////////////////////////////////////////////////
    //
    // Purpose: Drives the robot in reverse.
    //
    // Arguments:double distance, The distance to be traveled in inches.
    //
    // Returns: An int representing overshoot/undershoot of the movement
    // in inches.
    //
    // Remarks: It is assumed that the first time a movement is attempted
    // that it is a forward movement. The first movement appears
    // to determine the direction of the encoder counts. What
    // we observed during the last practice was that when we
    // started the program and the first motion was backwards
    // we saw increasing encoder counts. Assumed here is that
    // if the first movement on program start is forward then
    // reversing direction should create decreasing encoder
    // counts.
    // A possible option is to use the other encoder for reverse
    // motion. We shall see..
    //
    // 02/19/2022:  Modified for FalconFX
    //
    /////////////////////////////////////////////////////////////////////
    /////////////////////////////////////////////////////////////////////
    public double driveBwd(double distance) {
        double counts; // Encoder counts.
        double initial_position; // Our initial position.
        double current_position; // CUrrent position of the robot.
        double target; // Init position + encoder counts.

        double fraction; // How close we are to our target.
        double heading; // Initial gyro angle.

        double error; // Overshoot/undershoot.

        // Determine where we are pointing - we want to maintain this heading during
        // this forward movement.
        heading = driveGyro.getAngle();
        frontLeft.setSelectedSensorPosition(0.0); 

        // Read encoder #1, get the initial encoder position and assign it to
        // the current position. Calculate the number of encoder counts
        // necessary to reach the final destination (target).
        initial_position = frontLeft.getSelectedSensorPosition(); // should be zero
        current_position = initial_position;
        counts = calcCounts_SAE(distance);
        target = initial_position - counts;

        // fraction starts out as equal to 1.0 and decreases as we approach the target.
        // fraction is counts remaining divided by total counts.
        fraction = Math.abs((target - current_position) / (target - initial_position));

        System.out
                .println("initial_position = " + initial_position + " target = " + target + " fraction = " + fraction);

        // We attempt a bit of proportional control as we approach the target. We want
        // to slow down so that we don't overshoot. These fractions appear to work.
        // We drive at high speed for 80% of the distance and then slow. On carpet this
        // seemed to work very well for distance of 10 feet.
        // We want braking enabled.
        // We also need to put a timer within the while() loop to provide an escape in
        // the
        // event that the system gets lost during autonomous requiring a restart of the
        // program.

        // Need to test and determine sign of inequality. We had the curious behavior of
        // increasing encoder counts when moving backwards. It could have been due to
        // the fact that the first motion was backwards in our "spaghetti code". We will
        // Need to play around with the signs to get it working right.
        while (current_position > target) {
            if (fraction > BRAKE_FRACTION) {
                moveBwd(START_SPEED, heading);
            } else {
                moveBwd(BRAKE_SPEED, heading);
            }
            delay.delay_seconds(0.01);

            current_position = frontLeft.getSelectedSensorPosition(); 
            fraction = Math.abs((target - current_position) / (target - initial_position));
        }

        // stop movement, compute error (overshoot or undershoot)
        mecanumDrive.driveCartesian(0, 0, 0);
        current_position = frontLeft.getSelectedSensorPosition(); 
        error = calcDistance_SAE(current_position - target);
        System.out.println("error = " + error + " inches");

        return (error);
    }

       /////////////////////////////////////////////////////////////////////
    // Function: driveFwd_Inches(...)
    /////////////////////////////////////////////////////////////////////
    //
    // Purpose: Used for driving forward in autonomous.
    //
    // Arguments: double inches.
    //
    // Returns: void
    //
    // Remarks: Created on 12/08/2021.  Intended to be used to position
    // the robot based on camera measurements.
    // 02/19/2022:  Modified for FalconFX motor/encoder
    // 
    /////////////////////////////////////////////////////////////////////
    /////////////////////////////////////////////////////////////////////
    public void driveFwd_Inches(double inches) {

        double encoder_count;
        double target_count;

        // Initialize the encoder to 0 (reset it).
        frontLeft.setSelectedSensorPosition(0.0); 

        // Our current encoder count reading (should be zero).
        encoder_count = 0;

        // This should give us how many encoder counts.
    
        target_count = inches / encoder_precision;

        while (target_count < encoder_count) {

            // Drive forward.
            frontLeft.set(0.4);
            backLeft.set(-0.4);
            frontRight.set(0.4);
            backRight.set(-0.4);

            // Delay for 20 ms.
            delay.delay_milliseconds(20.0);

            // Read the encoder, and get our current count.
            encoder_count = Math.abs(frontLeft.getSelectedSensorPosition()); // should be zero);
        }

        // Stop the motors.
        fullStop();

    }

    /////////////////////////////////////////////////////////////////////
    // Function: driveBwd_Inches(...)
    /////////////////////////////////////////////////////////////////////
    //
    // Purpose: Used for driving reverse in autonomous.
    //
    // Arguments: double inches.
    //
    // Returns: void
    //
    // Remarks: Created on 12/08/2021.  Intended to be used to position
    // the robot based on camera measurements.  It is assumed that
    // negative readings will be realized with reverse motion.
    //
    //  02/19/2022:  Modified for FalconFX
    //
    // 
    /////////////////////////////////////////////////////////////////////
    /////////////////////////////////////////////////////////////////////
    public void driveBwd_Inches(double inches) {

        double encoder_count;
        double target_count;

        // Initialize the encoder to 0 (reset it).
        frontLeft.setSelectedSensorPosition(0.0); 
        // Our current encoder count reading (should be zero).
        encoder_count = 0;

        // This should give us how many encoder counts.
        target_count = inches / encoder_precision;

        while (target_count > encoder_count) {

            // Drive forward.
            frontLeft.set(-0.4);
            backLeft.set(0.4);
            frontRight.set(-0.4);
            backRight.set(0.4);

            // Delay for 20 ms.
            delay.delay_milliseconds(20.0);

            // Read the encoder, and get our current count.
            encoder_count = Math.abs(frontLeft.getSelectedSensorPosition()); // should be zero);
        }

        // Stop the motors.
        fullStop();

    }

    ///////////////////////////////////////////////////////////////////////////
    // Function: void moveBwd()
    ///////////////////////////////////////////////////////////////////////////
    //
    // Purpose: Uses on_board gyro to drive the robot straight backward
    // given a target angle as an argument.
    // Requires use of the arcadeDrive function with the first
    // argument being the forward speed and the second being
    // the turn applied.
    //
    // Arguments: double speed. Must be between -1.0 and 1.0.
    // double angle - the target angle.
    //
    // Returns: void
    //
    // Remarks:
    //  02/19/2022:  Modified for FalconFX
    //
    ////////////////////////////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////////////
    public void moveBwd(double speed, double target) {

        double corr = 0.2;
        double angle = 0;
        double delta; // The difference between the target and measured angle

        angle = driveGyro.getAngle();

        delta = angle - target;

        // According to the documentation for DifferentialDrive.arcadeDrive(speed,turn)
        // the arguments are squared to accommodate lower drive speeds. If for example
        // the gain coefficient is 0.05 and the angle error is 5 degrees, the turning
        // argument would be 0.25*0.25 = 0.0625. This is a pretty slow correction.
        // We needed a larger correction factor - trying 0.2 for now. The range for
        // the turn is -1.0 to 1.0. Positive values are said to turn clockwise,
        // negatives counterclockwise.
        mecanumDrive.driveCartesian(0, -speed, -corr * delta);

        // System.out.println(" target = " + target + " angle = " + angle + " delta = "
        // + delta);

    }


    ///////////////////////////////////////////////////////////////////////////
    // Function: calcCounts_SAE(double distance)
    ///////////////////////////////////////////////////////////////////////////
    //
    // Purpose: Computes the encoder readout corresponding to the submitted
    // distance.
    //
    // Arguments: Accepts a double representing the distance in feet.
    //
    // Returns: A double representing the encoder change associated
    // with that distance.
    //
    // Remarks: encoder_precision is the number of inches per
    // encoder count. The example is based on eight inch diameter wheels used
    // on the 2019 robot.
    //
    // Example: If we are traveling 10 inches (0.833 ft.), the expected change
    // in output from the encoder would be:
    //
    // 10.0/encoder_precision = 10395 counts  (eight inch wheels)
    //
    // 02/19/2022: Modified for FalconFX
    //
    ////////////////////////////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////////////
    public double calcCounts_SAE(double distance) {
        double enc_change;

        distance *= 12.0; // convert feet to inches.

        enc_change = distance/encoder_precision;

        return (enc_change);

    }

    // Given the counts, returns the distance in inches.
    public double calcDistance_SAE(double counts) {
        double distance;

        distance = counts * encoder_precision;

        return (distance);

    }

    ///////////////////////////////////////////////////////////////////////////
    // Function: void calcCounts_Metric(double radius,double distance)
    ///////////////////////////////////////////////////////////////////////////
    //
    // Purpose: Given the distance to be traveled in meters, this function
    // calculates the encoder change required to travel that distance.
    //
    // Arguments: double distance (in meters).
    //
    // Returns: A double representing the encoder change for specified distance.
    //
    // Remarks: 
    // 02/19/2022:  Modified for FalconFX
    //
    ////////////////////////////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////////////
    public double calcCounts_Metric(double distance) {
        double enc_change;

        distance *= 100.0; // convert meters to centimeters
        distance /= 2.54; // convert centimeters to inches

        enc_change = distance / encoder_precision;

        return (enc_change);

    }

    /////////////////////////////////////////////////////////////////////
    // Function: public double turnRight_Arcade(double degrees)
    /////////////////////////////////////////////////////////////////////
    //
    // Purpose: Rotates the robot CW through the angle specified
    // by degrees.
    //
    // Returns: A double representing the error.
    //
    // Remarks: Uses Arcade drive to rotate the robot. Note that we
    // want motor braking enabled. Added escape count to exit
    // loop after a certain time when it doesn't reach the target.
    //
    //
    /////////////////////////////////////////////////////////////////////
    /////////////////////////////////////////////////////////////////////
    public double turnRight_Arcade(double degrees) {
        int count = 0;
        double rot_speed = ROT_SPEED;
        double angle; // current gyro angle
        double target; // target angle
        double error;

        angle = driveGyro.getAngle(); // this is our starting point
        target = angle + degrees;
        System.out.println("ANGLE = " + angle + " Target = " + target);

        while (angle < target) {
            rotatePosArcade(rot_speed);
            angle = driveGyro.getAngle();

            if (angle > (target - ANGL_PROX_1)) {
                rot_speed /= ROT_ATTEN;
            }
            if (angle > (target - ANGL_PROX_2)) {
                rot_speed /= ROT_ATTEN;
            }
            delay.delay_seconds(0.01);
            count++;
            if ((count % GYRO_CONSOLE_UPDATE) == 0) {
                System.out.println("angle = " + angle);
            }
            if (count == GYRO_LOOP_ESCAPE)
                break;
        }
        mecanumDrive.driveCartesian(0, 0, 0);
        angle = driveGyro.getAngle();
        error = target - angle; // the error
        System.out.println("Turning Error = " + error + " degrees");
        return (error);
    }

    /////////////////////////////////////////////////////////////////////
    // Function: public void rotatePosArcade(double rot_spd)
    /////////////////////////////////////////////////////////////////////
    //
    // Purpose: Uses arcadeDrive(...) to rotate the robot CW
    //
    // Arguments:Accepts a double representing the rotation speed.
    //
    // Returns: void
    //
    // Remarks: According to the documentation of this function the
    // argument "rot_speed" is squared.
    //
    /////////////////////////////////////////////////////////////////////
    /////////////////////////////////////////////////////////////////////
    public int rotatePosArcade(double rot_spd) {

        if (rot_spd > 1.0) {
            return (-1);
        }
        mecanumDrive.driveCartesian(0, 0, rot_spd);
        return (0);
    }

    /////////////////////////////////////////////////////////////////////
    // Function: public double turnLeft_Arcade(double degrees)
    /////////////////////////////////////////////////////////////////////
    //
    // Purpose: Rotates the robot CCW through the specified number
    // of degrees.
    //
    // Arguments: The degrees of CCW rotation.
    //
    // Returns: A double representing the error.
    //
    // Remarks: Uses Arcade drive to
    // rotate the robot. Note that we want motor braking
    // enabled.
    //
    //
    // The final variable ROT_ATTEN value is yet to be determined but
    // for the first iteration it is 2.0.
    //
    /////////////////////////////////////////////////////////////////////
    /////////////////////////////////////////////////////////////////////
    public double turnLeft_Arcade(double degrees) {
        int count = 0;
        double rot_speed = ROT_SPEED;
        double angle; // current gyro angle
        double target;
        double error;

        // Current angle
        angle = driveGyro.getAngle();
        target = angle - degrees;

        while (angle > target) {
            rotateNegArcade(rot_speed);
            angle = driveGyro.getAngle();

            if (angle < (target + ANGL_PROX_1)) {
                rot_speed /= ROT_ATTEN;
            }
            if (angle < (target + ANGL_PROX_2)) {
                rot_speed /= ROT_ATTEN;
            }
            delay.delay_seconds(0.01);
            count++;
            if ((count % GYRO_CONSOLE_UPDATE) == 0) {
            }
            if (count == GYRO_LOOP_ESCAPE)
                break;
        }
        mecanumDrive.driveCartesian(0, 0, 0);
        angle = driveGyro.getAngle();
        System.out.println("angle = " + angle);
        error = target - angle;
        System.out.println("Turning Error = " + error + " degrees");
        return (error);

    }

    /////////////////////////////////////////////////////////////////////
    // Function: public int rotateNegArcade(double rot_spd)
    /////////////////////////////////////////////////////////////////////
    //
    // Purpose: Uses arcadeDrive(...) to rotate the robot CCW
    //
    // Arguments:Accepts a double representing the rotation speed.
    //
    // Returns: Zero normally, will return -1 if an unacceptable
    // rotation speed is entered
    //
    // Remarks: According to the documentation of this function the
    // argument "rot_speed" is squared.
    //
    /////////////////////////////////////////////////////////////////////
    /////////////////////////////////////////////////////////////////////
    public int rotateNegArcade(double rot_spd) {
        if (rot_spd > 1.0) {
            return (-1);
        }
        mecanumDrive.driveCartesian(0, 0, -rot_spd);
        return (0);
    }

    /////////////////////////////////////////////////////////////////////
    // Function: public double turnAbsolute(double degrees)
    /////////////////////////////////////////////////////////////////////
    //
    // Purpose: Rotate the number of degrees in the direction specified
    // by the sign of the argument "degrees". Negative arguments will
    // rotate CCW, positive arguments CW.
    //
    // Arguments: A double representing the number of degrees to rotate
    //
    //
    // Returns: A double representing the difference between the
    // achieved rotation and the requested rotation.
    //
    // Remarks: A few test cases:
    //
    // 1. Initial angle measurement is 110 degrees, we request a -35
    // degree rotation. Target is then 100-35=65 degrees. We
    // rotate ccw to 65 degrees.
    // 2. Initial angle measurement is -45 and we ask for 360. New
    // target is 315. We rotate cw all the way around to 315.
    //
    // Everything depends on the functions turnRight/Left_Arcade(...).
    // The braking algorithms will determine the accuracy.
    //
    /////////////////////////////////////////////////////////////////////
    /////////////////////////////////////////////////////////////////////
    public double turnAbsolute(double degrees) {

        // Rotation Direction Flags
        boolean ccw = false;
        boolean cw = false;

        double result = 0.0;

        if (degrees < 0.0) { // we will rotate ccw
            ccw = true;
            cw = false;
        } else { // we rotate cw
            cw = true;
            ccw = false;
        }

        degrees = Math.abs(degrees);
        if (cw == true) {
            result = turnRight_Arcade(degrees);
        } else if (ccw == true) {
            result = turnLeft_Arcade(degrees);
        }

        return (result);
    }

    /////////////////////////////////////////////////////////////////////
    // Function: public double turn2Heading(double heading)
    /////////////////////////////////////////////////////////////////////
    //
    // Purpose: Turns the robot to a compass heading (0->360).
    //
    // Arguments:Accepts a double representing the target heading
    //
    // Returns: The achieved heading or a ridiculous value in the
    // event that a negative heading is entered.
    //
    // Remarks:
    //
    /////////////////////////////////////////////////////////////////////
    /////////////////////////////////////////////////////////////////////
    public double turn2Heading(double heading) {
        double angle;
        double delta;
        double change;
        double result;

        // Heading represents a "compass" reading, i.e.,
        // an angle from zero to something less than 360 degrees.
        // Negative arguments are not allowed. Because we want
        // to return the achieved heading we do not allow
        // submission of negative arguments and should return
        // a negative double so that the user can recognize
        // an error.
        if ((heading < 0.0) || (heading > 360.0)) {
            System.out.println("Submitted arguments must be greater than zero and less than 360");
            return (-999.9);
        }

        // This function will return an angle between 0 and 360 degrees.
        angle = getHeading();

        delta = heading - angle; // number of degrees to turn

        System.out.println("angle = " + angle + " delta = " + delta);

        // Reduce delta to the smallest necessary rotation
        // It should be something less than 180.0 or greater
        // than -180.0
        if (delta > 180.0) {
            delta -= 360.0;
        }
        if (delta < -180.0) {
            delta += 360.0;
        }

        System.out.println("delta = " + delta);

        // Depending on the sign of delta we turn left (-) or right (+)
        change = turnAbsolute(delta);
        System.out.println("delta = " + delta + " change = " + change);

        // Measure the angle, convert to compass indication. Note that
        // reading of the gyro after a small rotation could still be
        // outside the range of compass indications. We want to put
        // it in the range of the compass (0->360)
        result = getHeading();
        return (result);
    }

    /////////////////////////////////////////////////////////////////////
    // Function: public double getHeading()
    /////////////////////////////////////////////////////////////////////
    //
    // Purpose: Utility routine to determine starting compass indication
    // of the robot.
    //
    // Arguments:none
    //
    // Returns: The current heading of the robot, 0->360 degrees.
    //
    // Remarks: Called twice within turn2Heading.
    //
    /////////////////////////////////////////////////////////////////////
    /////////////////////////////////////////////////////////////////////
    public double getHeading() {
        int debug = 1;
        double angle;

        // A heading represents a "compass" reading, i.e.,
        // an angle from zero to something less than 360 degrees.

        // In the event that the current gyroscope indication is greater than
        // 360 degrees, reduce it to something within the compass range.
        // The same argument applies if the initial indication is less
        // than -360.0 degrees.
        angle = driveGyro.getAngle();
        while (angle > 360.0) {
            angle -= 360.0;
            if (debug == 1) {
                System.out.println("Compass heading = " + angle);
            }
        }

        // In the event that the current gyroscope indication is less
        // than -360.0, increase it to something within the compass range.
        while (angle < -360.0) {
            angle += 360.0;
            if (debug == 1) {
                System.out.println("Compass heading = " + angle);
            }
        }
        System.out.println("Compass heading = " + angle);

        return (angle);
    }



   /////////////////////////////////////////////////////////////////
    //  Function:  int fullStop()
    /////////////////////////////////////////////////////////////////
    //
    //  Purpose:  Stops all drive motors
    //
    //  Arguments: void
    //  
    //  Returns:  Returns zero
    //
    ////////////////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////
    int fullStop()
    {
        // Stop the motors, because we're at our target.
        frontLeft.set(0);
        backLeft.set(0);
        frontRight.set(0);
        backRight.set(0);

        return(0);

    
    }



 
}





