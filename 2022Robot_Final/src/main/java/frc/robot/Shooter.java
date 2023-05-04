/////////////////////////////////////////////////////////////////////
//  File:  Shooter.java
/////////////////////////////////////////////////////////////////////
//
//  Purpose:  Source file creation of a java class containing
//            elements for shooting the ball.
//
//  Compiling Environment:  Microsoft Java VS
//
//  Remarks:  Created 3/6/22 
//
//  
/////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////

package frc.robot;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class Shooter extends Robot {

    // private Delay delay;

    WPI_TalonFX sweeper;
    WPI_TalonFX loader;
    CANSparkMax shooter;

    Shooter() {

        // Create the motors and enable the internal encoders.
        sweeper = new WPI_TalonFX(constants.sweeperID);

        loader = new WPI_TalonFX(constants.loaderID);

        shooter = new CANSparkMax(constants.shooterID, MotorType.kBrushless);

        sweeper.configFactoryDefault();
        loader.configFactoryDefault();
        shooter.restoreFactoryDefaults();

        sweeper.setNeutralMode(NeutralMode.Coast);
        loader.setNeutralMode(NeutralMode.Brake);
        shooter.setIdleMode(IdleMode.kBrake);

        shooter.setInverted(true);
        loader.setInverted(false);
        sweeper.setInverted(false);

        // Remaining
        // Which motors (if any) are inverted? Coast or brake mode?

    }

    public void shooterMotorsOff() {
        if (stick.getRawButton(constants.X) == false && stick.getRawButton(constants.Square) == false
                && stick.getRawButton(constants.Circle) == false && stick.getRawButton(constants.Triangle) == false) {
            sweeper.set(0);
            loader.set(0);
            shooter.set(0);
        }
    }

    public void Intake() {
        if (stick.getRawButton(constants.X) == true) {
            sweeper.set(0.40);
            loader.set(0.25);
            shooter.set(-0.1);
        }
    }

    public void ejectMotors() {
        if (stick.getRawButton(constants.Circle) == true) {
            loader.set(-1);
            shooter.set(-1);
            sweeper.set(-0.75);
        }
    }

    public void ShootLowPower() {
        if (stick.getRawButton(constants.Square) == true) {
            shooter.set(0.45);
        }
    }

    public void kickDown() {
        if (stick.getRawButton(constants.Triangle) == true) {
            shooter.set(-0.8);
            loader.set(-0.8);
            sweeper.set(0.4);
        }
    }

    public void dropIntake() {
        sweeper.set(1);
        delay.delay_seconds(1.5);
        sweeper.set(0);
    }

    public void autoShoot(double power) {
        shooter.set(power);
        delay.delay_seconds(0.5);
        shooter.set(power);
        loader.set(0.5);
        sweeper.set(0.5);
        delay.delay_seconds(1.5);
        shooter.set(0);
        loader.set(0);
        sweeper.set(0);
    }

    public void autoIntake(double time) {
        sweeper.set(0.5);
        loader.set(.25);
        delay.delay_seconds(time);
        shooterMotorsOffAuto();
    }

    public void autoIntakeCont() {
        sweeper.set(0.5);
        loader.set(.25);
    }

    public void shooterMotorsOffAuto() {
        sweeper.set(0);
        loader.set(0);
        shooter.set(0);
    }

    public void kickDownAuto() {
        sweeper.set(0.7);
        shooter.set(-0.7);
        loader.set(-0.5);
        delay.delay_seconds(0.2);
        shooterMotorsOffAuto();
    }

    public void kickDown2BallAuto() {
        sweeper.set(0.7);
        shooter.set(-0.5);
        loader.set(-0.3);
        delay.delay_seconds(0.4);
        shooterMotorsOffAuto();
    }
}
