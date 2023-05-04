package frc.robot;

public class Constants extends Robot {
    // Playstation Axis IDs
    final int Left_X_Axis = 0;
    final int Left_Y_Axis = 1;
    final int Right_X_Axis = 2;
    final int Right_Y_Axis = 5;
    final int Left_Trigger = 3;
    final int Right_Trigger = 4;

    // Playstation Button IDs
    final int Square = 1;
    final int X = 2;
    final int Circle = 3;
    final int Triangle = 4;
    final int Left_Bumper = 5;
    final int Right_Bumper = 6;
    final int Share_Button = 9;
    final int Options_Button = 10;
    final int L3 = 11;
    final int R3 = 12;
    final int PS = 13;
    final int Touchpad = 14;

    // Motor IDs
    final int frontLeftDrive = 1;
    final int frontRightDrive = 2;
    final int backLeftDrive = 3;
    final int backRightDrive = 4;
    final int liftMotor = 5;
    final int climbMotor1 = 6;
    final int climbMotor2 = 7;

    //  Shooting System CAN IDs
    final int sweeperID=8;
    final int loaderID=9;
    final int shooterID=10;

    // Pneumatics IDs
    final int midRungIn = 4; //correct
    final int midRungOut = 5; //correct
    final int highRungIn = 3; //correct
    final int highRungOut = 2; //correct
    final int travRungIn = 0;
    final int travRungOut = 1;

    final double climb_deadband = 50.0;
}
