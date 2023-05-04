
/////////////////////////////////////////////////////////////////////
//  File:  Climb.java
/////////////////////////////////////////////////////////////////////
//
//  Purpose:  Source file creation of a java class containing
//            elements for controlling the climb.
//
//  Compiling Environment:  Microsoft Java VS
//
//  Remarks:  Modified 3/6/22 to simplify processing buttons for
//            the pneumatic cylinders. Also enabled TalonFX
//            motor encoders.  In the future we may want to
//            use the encoders for targeted movements and
//            limits.
//
//  
/////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////

package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.PneumaticsModuleType;

public class Climb extends Robot {

    // Climb Motors
    WPI_TalonFX climbMotor1;
    WPI_TalonFX climbMotor2;
    WPI_TalonFX liftMotor;

    Compressor comp;

    PneumaticHub pHub;

    // Double solenoids
    DoubleSolenoid midRung;
    DoubleSolenoid highRung;
    DoubleSolenoid travRung;

    // Solenoid parameters
    int midGrab_state;
    int highGrab_state;
    int travGrab_state;
    int climbMode_state;

    // State is either extended or retracted
    private final int extended = 1;
    private final int retracted = 2;

    Boolean midGrab_enabled;
    Boolean highGrab_enabled;
    Boolean travGrab_enabled;
    Boolean runClimbMode;

    double climbMotorEncoder1;
    double climbMotorEncoder2;

    Climb() {

        climbMotor1 = new WPI_TalonFX(constants.climbMotor1);
        climbMotor2 = new WPI_TalonFX(constants.climbMotor2);
        liftMotor = new WPI_TalonFX(constants.liftMotor);

        climbMotor1.configFactoryDefault();
        climbMotor2.configFactoryDefault();
        liftMotor.configFactoryDefault();

        climbMotor1.setNeutralMode(NeutralMode.Brake);
        climbMotor2.setNeutralMode(NeutralMode.Brake);
        liftMotor.setNeutralMode(NeutralMode.Brake);

        climbMotor1.setInverted(false);
        climbMotor2.setInverted(true);
        liftMotor.setInverted(false);

        climbMotor1.setSafetyEnabled(false);
        climbMotor2.setSafetyEnabled(false);
        liftMotor.setSafetyEnabled(false);

        climbMotor1.configNeutralDeadband(0.09);
        climbMotor2.configNeutralDeadband(0.09);
        liftMotor.configNeutralDeadband(0.06);

        climbMotor1.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 50);
        climbMotor2.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 50);

        climbMotorEncoder1 = climbMotor1.getSelectedSensorPosition();
        climbMotorEncoder2 = climbMotor2.getSelectedSensorPosition();

        pHub = new PneumaticHub(8);

        comp = new Compressor(8, PneumaticsModuleType.REVPH);

        midRung = new DoubleSolenoid(8, PneumaticsModuleType.REVPH,
                constants.midRungOut, constants.midRungIn);
        highRung = new DoubleSolenoid(8, PneumaticsModuleType.REVPH,
                constants.highRungOut, constants.highRungIn);
        travRung = new DoubleSolenoid(8, PneumaticsModuleType.REVPH,
                constants.travRungOut, constants.travRungIn);

        // Set all cylinders in the retracted position to start;
        // set the state of each accordingly. Enable buttons.
        midGrab_enabled = true;
        highGrab_enabled = true;
        travGrab_enabled = true;

        runClimbMode = false;

        midGrab_state = retracted;
        highGrab_state = retracted;
        travGrab_state = retracted;
        climbMode_state = retracted;

        // Make sure cylinders start in retracted state.
        midRung.set(Value.kReverse);
        highRung.set(Value.kReverse);
        travRung.set(Value.kReverse);

        comp.enableAnalog(115, 120);
        // pHub.disableCompressor();
    }

    public void rotateClimb() {
        climbMotor1.set(ControlMode.PercentOutput, -stick.getRawAxis(constants.Right_Y_Axis) * .5);
        climbMotor2.follow(climbMotor1);
    }

    public void turnClimbMotorsOff() {
        if(runClimbMode == false) {
            climbMotor1.set(0);
            climbMotor2.set(0);
        }
    }

    public void runLiftClimbMode() {
        if (stick.getRawButton(constants.Triangle) == true) {
            liftMotor.set(ControlMode.PercentOutput, 1);
        }
        if (stick.getRawButton(constants.Circle) == true) {
            liftMotor.set(ControlMode.PercentOutput, -1);
        } else if (stick.getRawButton(constants.Triangle) == false
                && stick.getRawButton(constants.Circle) == false) {
            liftMotor.set(ControlMode.PercentOutput, 0);
        }
    }

    public void runLiftNormalMode() {
        liftMotor.set(ControlMode.PercentOutput, -stick.getRawAxis(constants.Right_Y_Axis));
    }

    public void climbGrab() {

        processMidGrab(constants.Right_Trigger_Button);

        processHighGrab(constants.Left_Trigger_Button);

        processTravGrab(constants.Right_Bumper);

    }

    ///////////////////////////////////////////////////////////////
    // Pnuematic Processing Functions
    ///////////////////////////////////////////////////////////////
    // We need to know what we expect to happen the first time the
    // button press is detected. Do we expect the first move to
    // be extension or retraction? Here I assume that the first
    // operation will be to extend the cylinder. I do this by
    // initializing the state to "retracted" in the teleOPInit()
    // function and forcing movement to retracted.
    //
    // Here is the logic:
    // We want to keep a history of the last button press, i.e., the
    // previous state. We do not enable the button unless it is first
    // detected that it has been released. The exception to this is
    // the very first button press. We have enabled the button in the
    // TeleopInit() function.
    // The functions getRawButton() are used to determine the
    // state of the button when the loop returns (approx every
    // 20msec). A delay must be added to at least allow some
    // time to elapse before we read the button status.
    // In the present configuration a 20msec delay is added after
    // each detection. This may be able to be shortened but on
    // the test board this works.
    //
    // 1. We pressed it the first time, assumed that this means
    // to extend the cylinder.
    // 2. As a button press is detected, we disable actions
    // associated with further presses of the button until a
    // release has been detected.
    // 3. Use of the previous value of state and the "if/else if"
    // logic facilitates the alternate operations associated
    // with multiple presses of the button.
    // 4. Disabling the button after each action allows detection
    // of the release and prevents rapid alternate actions.
    // 5. Release detection enables the button but does not
    // change the previous state, allowing alternating button
    // press operations.
    // 6. Note that this "spaghetti code" could be put in a
    // function that had, as arguments, the associated
    // button and the specific double solenoid/cylinder.
    // 3/6/22: Done, see below.

    /////////////////////////////////////////////////////////////////////
    // Function: int processMidGrab(int button)
    /////////////////////////////////////////////////////////////////////
    //
    // Purpose: Processes consecutive button presses associated with
    // the midRung double solenoid and cylinders.
    //
    // Arguments:Accepts an int corresponding to the joystick button
    // assignment.
    //
    // Returns: Zero. Could be modified in the future to return
    // an error code.
    //
    // Remarks: 3/6/22: Routine worked on the test board, untested
    // at this point on the robot.
    //
    //
    /////////////////////////////////////////////////////////////////////
    /////////////////////////////////////////////////////////////////////
    int processMidGrab(int button) {
        if ((stick.getRawButton(button) == true)
                && (midGrab_state == retracted) && (midGrab_enabled == true)) {
            midRung.set(Value.kForward);
            midGrab_state = extended;
            midGrab_enabled = false;
            System.out.println("Button Pressed, State = " + midGrab_state);
            delay.delay_milliseconds(20);
        } else if ((stick.getRawButton(button) == true)
                && (midGrab_state == extended) && (midGrab_enabled == true)) {
            System.out.println("Button Pressed");
            midRung.set(Value.kReverse);
            midGrab_state = retracted;
            midGrab_enabled = false;
            System.out.println("Button Pressed, State = " + midGrab_state);
            delay.delay_milliseconds(20);
        } else if ((stick.getRawButton(button) == false)
                && (midGrab_enabled == false)) {
            System.out.println("Button Released, State = " + midGrab_state);
            midGrab_enabled = true;
            delay.delay_milliseconds(20);
        }
        return (0);
    }

    /////////////////////////////////////////////////////////////////////
    // Function: int processHighGrab(int button)
    /////////////////////////////////////////////////////////////////////
    //
    // Purpose: Processes consecutive button presses associated with
    // the highRung double solenoid and cylinders.
    //
    // Arguments:Accepts an int corresponding to the joystick button
    // assignment.
    //
    // Returns: Zero. Could be modified in the future to return
    // an error code.
    //
    // Remarks: 3/6/22: Routine worked on the test board, untested
    // at this point on the robot.
    //
    //
    /////////////////////////////////////////////////////////////////////
    /////////////////////////////////////////////////////////////////////
    int processHighGrab(int button) {
        if ((stick.getRawButton(button) == true)
                && (highGrab_state == retracted) && (highGrab_enabled == true)) {
            highRung.set(Value.kForward);
            highGrab_state = extended;
            highGrab_enabled = false;
            System.out.println("Button Pressed, State = " + highGrab_state);
            delay.delay_milliseconds(20);
        } else if ((stick.getRawButton(button) == true)
                && (highGrab_state == extended) && (highGrab_enabled == true)) {
            System.out.println("Button Pressed");
            highRung.set(Value.kReverse);
            highGrab_state = retracted;
            highGrab_enabled = false;
            System.out.println("Button Pressed, State = " + highGrab_state);
            delay.delay_milliseconds(20);
        } else if ((stick.getRawButton(button) == false)
                && (highGrab_enabled == false)) {
            System.out.println("Button Released, State = " + highGrab_state);
            highGrab_enabled = true;
            delay.delay_milliseconds(20);
        }
        return (0);
    }

    /////////////////////////////////////////////////////////////////////
    // Function: int processTravGrab(int button)
    /////////////////////////////////////////////////////////////////////
    //
    // Purpose: Processes consecutive button presses associated with
    // the travRung double solenoid and cylinders.
    //
    // Arguments:Accepts an int corresponding to the joystick button
    // assignment.
    //
    // Returns: Zero. Could be modified in the future to return
    // an error code.
    //
    // Remarks: 3/6/22: Routine worked on the test board, untested
    // at this point on the robot.
    //
    //
    /////////////////////////////////////////////////////////////////////
    /////////////////////////////////////////////////////////////////////
    int processTravGrab(int button) {
        if ((stick.getRawButton(button) == true)
                && (travGrab_state == retracted) && (travGrab_enabled == true)) {
            travRung.set(Value.kForward);
            travGrab_state = extended;
            travGrab_enabled = false;
            System.out.println("Button Pressed, State = " + travGrab_state);
            delay.delay_milliseconds(20);
        } else if ((stick.getRawButton(button) == true)
                && (travGrab_state == extended) && (travGrab_enabled == true)) {
            System.out.println("Button Pressed");
            travRung.set(Value.kReverse);
            travGrab_state = retracted;
            travGrab_enabled = false;
            System.out.println("Button Pressed, State = " + travGrab_state);
            delay.delay_milliseconds(20);
        } else if ((stick.getRawButton(button) == false)
                && (travGrab_enabled == false)) {
            System.out.println("Button Released, State = " + travGrab_state);
            travGrab_enabled = true;
            delay.delay_milliseconds(20);
        }
        return (0);
    }

    public void toggleClimbMode(int button) {
        if (runClimbMode == false) {
            while (stick.getRawButton(button) == true) {
                runClimbMode = true;
            }
        }
        if (runClimbMode == true) {
            while (stick.getRawButton(button) == true) {
                runClimbMode = false;
            }
        }
    }

    public void climbAlign(int button) {
        double encoder1;
        double encoder2;
        encoder1 = climbMotor1.getSelectedSensorPosition();
        encoder2 = climbMotor2.getSelectedSensorPosition();

        while (stick.getRawButton(button) == true) {
            while (encoder1 > encoder2) {
                climbMotor1.set(-0.15);
                climbMotor2.set(0.15);
                encoder1 = climbMotor1.getSelectedSensorPosition();
                encoder2 = climbMotor2.getSelectedSensorPosition();
            }
            while (encoder2 > encoder1) {
                climbMotor1.set(0.15);
                climbMotor2.set(-0.15);
                encoder1 = climbMotor1.getSelectedSensorPosition();
                encoder2 = climbMotor2.getSelectedSensorPosition();
            }
        }

    }
}