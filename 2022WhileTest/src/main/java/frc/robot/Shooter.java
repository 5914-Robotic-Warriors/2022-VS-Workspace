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

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxRelativeEncoder;

public class Shooter extends Robot {

    // private Delay delay;

    CANSparkMax sweeper;
    CANSparkMax loader;
    CANSparkMax shooter;

    RelativeEncoder sweeper_encoder;
    RelativeEncoder loader_encoder;
    RelativeEncoder shooter_encoder;

    Shooter() {

        // Create the motors and enable the internal encoders.
        sweeper = new CANSparkMax(constants.sweeperID, MotorType.kBrushless);
        sweeper_encoder = sweeper.getEncoder(SparkMaxRelativeEncoder.Type.kHallSensor, 42);
        sweeper_encoder.setPosition(0.0);

        loader = new CANSparkMax(constants.loaderID, MotorType.kBrushless);
        loader_encoder = sweeper.getEncoder(SparkMaxRelativeEncoder.Type.kHallSensor, 42);
        loader_encoder.setPosition(0.0);

        shooter = new CANSparkMax(constants.shooterID, MotorType.kBrushless);
        shooter_encoder = sweeper.getEncoder(SparkMaxRelativeEncoder.Type.kHallSensor, 42);
        shooter_encoder.setPosition(0.0);

        sweeper.restoreFactoryDefaults();
        loader.restoreFactoryDefaults();
        shooter.restoreFactoryDefaults();

        sweeper.setIdleMode(IdleMode.kBrake);
        loader.setIdleMode(IdleMode.kBrake);
        shooter.setIdleMode(IdleMode.kBrake);

        shooter.setInverted(true);
        loader.setInverted(false);
        sweeper.setInverted(false);

        // Remaining
        // Which motors (if any) are inverted? Coast or brake mode?

    }

    public void shooterMotorsOff() {
        if (stick.getRawButton(constants.X) == false && stick.getRawButton(constants.Square) == false
                && stick.getRawButton(constants.Touchpad) == false) {
            sweeper.set(0);
            loader.set(0);
            shooter.set(0);
        }
    }

    public void Intake() {
        if (stick.getRawButton(constants.X) == true) {
            sweeper.set(0.5);
            loader.set(.25);
        }
    }

    public void ShootLowPower() {
        if (stick.getRawButton(constants.Square) == true) {
            shooter.set(0.35);
        }
    }

    public void ShootHighPower() {
        if (stick.getRawButton(constants.Circle) == true) {
            shooter.set(0.60);
        }
    }

    

    public void kickDown() {
        if (stick.getRawButton(constants.Triangle) == true) {
            shooter.set(-0.2);
            loader.set(-0.3);
            sweeper.set(0.25);
        }
    }

    public void dropIntake() {
        sweeper.set(1);
        delay.delay_seconds(0.5);
        sweeper.set(0);
    }

    public void autoShoot(double power) {
        shooter.set(power);
        delay.delay_seconds(0.5);
        loader.set(0.5);
        delay.delay_seconds(0.5);
        shooter.set(0);
        loader.set(0);
    }

    public void autoIntake(double time) {
        sweeper.set(0.6);
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
        sweeper.set(0.3);
        shooter.set(-0.5);
        loader.set(-0.2);
        delay.delay_seconds(0.2);
        shooterMotorsOffAuto();
    }
}
