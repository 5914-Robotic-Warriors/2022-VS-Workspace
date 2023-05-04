package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

//This class is for the intake and the shooter

public class Intake extends Robot {

    CANSparkMax shootLow;
    CANSparkMax shootHigh;
    CANSparkMax intakeMotor;

    Intake() {

        // shootLow = new CANSparkMax(constants.shootLow, MotorType.kBrushless);
        // shootHigh = new CANSparkMax(constants.shootHigh, MotorType.kBrushless);
        // intakeMotor = new CANSparkMax(constants.intakeMotor, MotorType.kBrushless);

        shootLow = new CANSparkMax(constants.shootLow, MotorType.kBrushless);
        shootHigh = new CANSparkMax(constants.shootHigh, MotorType.kBrushless);
        intakeMotor = new CANSparkMax(constants.intakeMotor, MotorType.kBrushless);

        shootHigh.setInverted(true);
        shootLow.setInverted(false);
        intakeMotor.setInverted(false);

        shootLow.setIdleMode(IdleMode.kBrake);
        shootHigh.setIdleMode(IdleMode.kCoast);
        intakeMotor.setIdleMode(IdleMode.kCoast);

    }

    public void Shoot() {
        if (stick.getRawButton(constants.Touchpad) == true) {
            shootHigh.set(.5);
        } else {
            shootHigh.set(0);
        }
    }

    public void kickDown() {
        if (stick.getRawButton(constants.PS) == true) {
            shootHigh.set(-0.1);
            shootLow.set(-0.1);
        }
    }

    public void intakeBall() {
        if (stick.getRawButton(constants.X) == true) {
            intakeMotor.set(0.5);
            shootLow.set(0.15);
        } else {
            shootLow.set(0);
            intakeMotor.set(0);
        }
    }
}
