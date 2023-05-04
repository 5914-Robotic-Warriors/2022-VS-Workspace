package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FollowerType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.PneumaticsModuleType;

public class Climb extends Robot {

    Compressor comp;

    DoubleSolenoid midRung;
    DoubleSolenoid highRung;
    DoubleSolenoid travRung;

    WPI_TalonFX climbMotor1;
    WPI_TalonFX climbMotor2;
    WPI_TalonFX liftMotor;

    boolean midGrab;
    boolean highGrab;
    boolean travGrab;

    PneumaticHub pHub;

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

        climbMotor2.setInverted(true);

        pHub = new PneumaticHub(8);

        comp = new Compressor(PneumaticsModuleType.REVPH);

        midRung = new DoubleSolenoid(8, PneumaticsModuleType.REVPH,
                constants.midRungOut, constants.midRungIn);
        highRung = new DoubleSolenoid(8, PneumaticsModuleType.REVPH,
                constants.highRungOut, constants.highRungIn);
        travRung = new DoubleSolenoid(8, PneumaticsModuleType.REVPH,
                constants.travRungOut, constants.travRungIn);

        midGrab = false;
        highGrab = false;
        travGrab = false;

        comp.enableAnalog(113, 120);
    }

    public void rotateClimb() {
        climbMotor1.set(ControlMode.PercentOutput, stick.getRawAxis(constants.Right_Y_Axis) / 2);
        climbMotor2.follow(climbMotor1, FollowerType.PercentOutput);
    }

    public void runLift() {
        if (stick.getRawButton(constants.Triangle) == true) {
            liftMotor.set(ControlMode.PercentOutput, .8);
        }
        if (stick.getRawButton(constants.Circle) == true) {
            liftMotor.set(ControlMode.PercentOutput, -0.8);
        } else if (stick.getRawButton(constants.Triangle) == false
                && stick.getRawButton(constants.Circle) == false) {
            liftMotor.set(ControlMode.PercentOutput, 0);
        }
    }

    public void climbGrab() {

        if (midGrab == true) {
            midRung.set(Value.kReverse);
        }
        if (midGrab == false) {
            midRung.set(Value.kForward);
        }

        if (highGrab == true) {
            highRung.set(Value.kReverse);
        }
        if (highGrab == false) {
            highRung.set(Value.kForward);
        }

        if (travGrab == true) {
            travRung.set(Value.kReverse);
        }
        if (travGrab == false) {
            travRung.set(Value.kForward);
        }

        if (midGrab == false) {
            while (stick.getRawButton(constants.Square) == true) {
                midGrab = true;
            }
        }
        if (midGrab == true) {
            while (stick.getRawButton(constants.Square) == true) {
                midGrab = false;
            }
        }

        if (highGrab == false) {
            while (stick.getRawButton(constants.Left_Bumper) == true) {
                highGrab = true;
            }
        }
        if (highGrab == true) {
            while (stick.getRawButton(constants.Left_Bumper) == true) {
                highGrab = false;
            }
        }

        if (travGrab == false) {
            while (stick.getRawButton(constants.Right_Bumper) == true) {
                travGrab = true;
            }
        }
        if (travGrab == true) {
            while (stick.getRawButton(constants.Right_Bumper) == true) {
                travGrab = false;
            }
        }
    }
}