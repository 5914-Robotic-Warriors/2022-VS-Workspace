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
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxRelativeEncoder;
import com.revrobotics.RelativeEncoder;

public class Shooter extends Robot  {

    //private Delay delay;

    CANSparkMax sweeper;
    CANSparkMax loader;
    CANSparkMax shooter;

    RelativeEncoder sweeper_encoder;
    RelativeEncoder loader_encoder;
    RelativeEncoder shooter_encoder;

    Shooter() 
    {

        //  Create the motors and enable the internal encoders.
        sweeper = new CANSparkMax(constants.sweeperID, MotorType.kBrushless);
        sweeper_encoder= sweeper.getEncoder(SparkMaxRelativeEncoder.Type.kHallSensor,42);
        sweeper_encoder.setPosition(0.0);

        loader = new CANSparkMax(constants.loaderID, MotorType.kBrushless);
        loader_encoder= sweeper.getEncoder(SparkMaxRelativeEncoder.Type.kHallSensor,42);
        loader_encoder.setPosition(0.0);

        shooter = new CANSparkMax(constants.shooterID, MotorType.kBrushless);
        shooter_encoder= sweeper.getEncoder(SparkMaxRelativeEncoder.Type.kHallSensor,42);
        shooter_encoder.setPosition(0.0);

        //  Remaining
        //  Which motors (if any) are inverted?  Coast or brake mode?

    }

    //  Create functions specific to the operations, loading shooting, etc..
    
}
