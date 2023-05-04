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

class Drive extends Robot {

    /*
     * Drive Encoder Parameters: updated 2/19/2022
     * Drive encoders are on the FalconFX drive motors. The output from the encoders
     * is
     * 2048 counts per shaft revolution. A gear reduction of 12.75:1 would imply
     * 12.75
     * times 2048 per revolution of the output shaft (26112 counts per output shaft
     * revolution).
     * An 8 inch wheel diameter would imply a distance traveled of PI*8.0 = 25.13
     * inches.
     * Distance precision of the encoder/gearbox combination is 25.13/26112 =
     * 9.62e-4
     * inches per count. Specification of gear reduction and wheel diameter as well
     * as the CAN addresses of the drive motors is in the Constants class.
     * 
     * Encoder position is read from a CANEncoder object by calling the
     * frontLeft.getSelectedSensorPosition(); method.
     * 
     * At present only one encoder is used. This can be changed if required
     */
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
    // private Delay delay;

    ADXRS450_Gyro driveGyro;

    Constants constants;

    boolean driveInverted = false;

    // Constructor for MecDrive()
    Drive() {

        delay = new Delay();

        constants = new Constants();

        // Creating the MecanumDrive constructor, which links all 4 motors together.

        // Constructing the motors, giving them their IDs, and making them brushless.
        frontLeft = new WPI_TalonFX(constants.frontLeftDrive);
        frontRight = new WPI_TalonFX(constants.frontRightDrive);
        backLeft = new WPI_TalonFX(constants.backLeftDrive);
        backRight = new WPI_TalonFX(constants.backRightDrive);

        // Invert the right side motors because we need it to work with the magic
        // function
        frontRight.setInverted(true);
        backRight.setInverted(true);
        frontLeft.setInverted(false);
        backLeft.setInverted(false);

        mecanumDrive = new MecanumDrive(frontLeft, backLeft, frontRight, backRight);
        mecanumDrive.setDeadband(0.05);
        mecanumDrive.setSafetyEnabled(false);

        // Encoder for the front left drive motor.
        frontLeft.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 50);

        backRight.setSelectedSensorPosition(0.0);
        frontLeftEncoderPosition = backRight.getSelectedSensorPosition();

        // Gyro for auto drive functions.
        driveGyro = new ADXRS450_Gyro();

        // Set the drive motors to coast mode to help prevent tipping,
        // and to make the drive less jerky.

        // We want to establish an initial encoder reading. This will enable resetting
        // encoder position to zero when we start moving. We use absolute values to
        // make the subsequent subtraction more easily interpreted.
        frontLeft.setSelectedSensorPosition(0.0);
        frontLeft.getSelectedSensorPosition(); // should be zero

        // Setup the initial heading
        initDriveGyroAngle = driveGyro.getAngle();
    }

    public void invertDrive(int button) {
        if (driveInverted == false) {
            while (stick.getRawButton(button) == true) {
                driveInverted = true;
            }
        }
        if (driveInverted == true) {
            while (stick.getRawButton(button) == true) {
                driveInverted = false;
            }
        }
    }

    public void invertedDriveMotors() {
        if (driveInverted == true) {
            // add the triggers into one axis so the driveCartesian program accepts it
            double triggerAddition;

            // divide by 2 in order to increase precision and eliminate getting -2 and 2 as
            // max values, instead getting -1 and 1
            triggerAddition = (-stick.getRawAxis(constants.Left_Trigger) / 2
                    + stick.getRawAxis(constants.Right_Trigger) / 2);

            // Use the second input for lateral movement, first input for forward
            // movement, and the third input for rotation.
            mecanumDrive.driveCartesian(-stick.getRawAxis(constants.Left_Y_Axis), triggerAddition,
                    stick.getRawAxis(constants.Left_X_Axis), 0.0);
        }
    }

    public void invertedDriveMotorsNoStrafe() {
        if (driveInverted == true) {
        mecanumDrive.driveCartesian(-stick.getRawAxis(constants.Left_Y_Axis), 0,
                stick.getRawAxis(constants.Left_X_Axis));
        }
    }

    public void standardDriveMotors() {
        if (driveInverted == false) {
            // add the triggers into one axis so the driveCartesian program accepts it
            double triggerAddition;

            // divide by 2 in order to increase precision and eliminate getting -2 and 2 as
            // max values, instead getting -1 and 1
            triggerAddition = (-stick.getRawAxis(constants.Left_Trigger) / 2
                    + stick.getRawAxis(constants.Right_Trigger) / 2);

            // Use the second input for lateral movement, first input for forward
            // movement, and the third input for rotation.
            mecanumDrive.driveCartesian(stick.getRawAxis(constants.Left_Y_Axis), -triggerAddition,
                    stick.getRawAxis(constants.Left_X_Axis), 0.0);
        }
    }

    public void standardDriveMotorsNoStrafe() {
        if (driveInverted == false) {
            mecanumDrive.driveCartesian(stick.getRawAxis(constants.Left_Y_Axis), 0,
                    stick.getRawAxis(constants.Left_X_Axis));
        }
    }

    public void driveToTargetBwd(double countTarget) {
        double distance;
        double target;

        distance = backRight.getSelectedSensorPosition();
        target = countTarget + distance;
        // countTarget = 16676; //1 foot
        // countTarget = -78846.0; // -5 feet
        // countTarget = 247077.0; // 15 feet

        while (distance < target) {
            distance = backRight.getSelectedSensorPosition();
            mecanumDrive.driveCartesian(0.3, 0, 0);
            // System.out.println("Encoder pos:" + backRight.getSelectedSensorPosition());
        }

        mecanumDrive.driveCartesian(0, 0, 0);

    }

    public void driveToTargetFwd(double countTarget) {
        double distance;
        double target;

        distance = backRight.getSelectedSensorPosition();
        target = countTarget + distance;
        // countTarget = 16676; //1 foot
        // countTarget = r78846.0; // 5 feet
        // countTarget = 247077.0; // 15 feet

        while (distance > target) {
            distance = backRight.getSelectedSensorPosition();
            mecanumDrive.driveCartesian(-0.3, 0, 0);
            // System.out.println("Encoder pos:" + backRight.getSelectedSensorPosition());
        }

        mecanumDrive.driveCartesian(0, 0, 0);

    }

    public void driveToTargetFwdwIntake(double countTarget) {
        double distance;
        double target;

        distance = backRight.getSelectedSensorPosition();
        target = countTarget + distance;
        // countTarget = 16676; //1 foot
        // countTarget = r78846.0; // 5 feet
        // countTarget = 247077.0; // 15 feet

        while (distance > target) {
            distance = backRight.getSelectedSensorPosition();
            mecanumDrive.driveCartesian(-0.3, 0, 0);
            shooter.autoIntakeCont();
        }
        shooter.shooterMotorsOffAuto();

        mecanumDrive.driveCartesian(0, 0, 0);

    }

    public void driveToTargetBwdwIntake(double countTarget) {
        double distance;
        double target;

        distance = backRight.getSelectedSensorPosition();
        target = countTarget + distance;
        // countTarget = 16676; //1 foot
        // countTarget = -78846.0; // -5 feet
        // countTarget = 247077.0; // 15 feet

        while (distance < target) {
            distance = backRight.getSelectedSensorPosition();
            mecanumDrive.driveCartesian(0.3, 0, 0);
            shooter.autoIntakeCont();
        }
        shooter.shooterMotorsOffAuto();

        mecanumDrive.driveCartesian(0, 0, 0);

    }

    public void turnRight(double angle) {
        double target;
        double gyro;

        target = driveGyro.getAngle() + angle;
        gyro = driveGyro.getAngle();

        while (gyro < target) {
            mecanumDrive.driveCartesian(0, 0, 0.2);
            gyro = driveGyro.getAngle();
        }
        mecanumDrive.driveCartesian(0, 0, 0);
    }

    public void turnLeft(double angle) {
        double target;
        double gyro;
        angle = angle * -1;

        target = driveGyro.getAngle() + angle;
        gyro = driveGyro.getAngle();

        while (gyro > target) {
            mecanumDrive.driveCartesian(0, 0, -0.2);
            gyro = driveGyro.getAngle();
        }
        mecanumDrive.driveCartesian(0, 0, 0);
    }

    public void turnRightwIntake(double angle) {
        double target;
        double gyro;

        target = driveGyro.getAngle() + angle;
        gyro = driveGyro.getAngle();

        while (gyro < target) {
            mecanumDrive.driveCartesian(0, 0, 0.2);
            gyro = driveGyro.getAngle();
            shooter.autoIntakeCont();
        }
        shooter.shooterMotorsOffAuto();
        mecanumDrive.driveCartesian(0, 0, 0);
    }

    public void turnLeftwIntake(double angle) {
        double target;
        double gyro;
        angle = angle * -1;

        target = driveGyro.getAngle() + angle;
        gyro = driveGyro.getAngle();

        while (gyro > target) {
            mecanumDrive.driveCartesian(0, 0, -0.2);
            gyro = driveGyro.getAngle();
            shooter.autoIntakeCont();
        }
        shooter.shooterMotorsOffAuto();
        mecanumDrive.driveCartesian(0, 0, 0);
    }
}