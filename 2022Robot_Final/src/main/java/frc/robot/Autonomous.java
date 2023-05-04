package frc.robot;

public class Autonomous extends Robot {

    Autonomous() {

    }

    public void driveOutOfAreaFwd() {
        drive.driveToTargetFwd(-100000.0);
    }

    public void driveOutOfAreaBwd() {
        drive.driveToTargetBwd(100000.0);
    }

    public void wallLeft2Ball() {
        shooter.dropIntake();
        shooter.autoShoot(0.40);
        drive.driveToTargetBwd(98186.0); // this goes from the wall to pushing the red ball out of the way
        drive.turnRight(75.0); // turns towards blue ball
        drive.driveToTargetFwdwIntake(-74000.0); // this drives towards the blue ball while intaking
        shooter.autoIntake(0.4);
        drive.turnLeftwIntake(100); // turns left back towards goal
        drive.driveToTargetFwd(-110000.0); // drives towards goal
        shooter.kickDownAuto(); // kicks ball down
        shooter.autoShoot(0.40); // shoots
    }

    public void wallLeft1Ball() {
        shooter.dropIntake();
        shooter.autoShoot(0.45);
        drive.driveToTargetBwd(98186.0); // this goes from the wall to pushing the red ball out of the way
        drive.turnRight(75.0); // turns towards blue ball
    }

    public void middleWallRightSide2Ball() {
        shooter.dropIntake();
        drive.driveToTargetFwdwIntake(-83290.0);
        drive.turnLeftwIntake(175);
        drive.driveToTargetFwd(-65000);
        drive.turnLeft(10);
        shooter.kickDown2BallAuto();
        shooter.autoShoot(0.4);
    }

    public void middleWallRightSide1Ball() {
        drive.driveToTargetFwd(-88000.0);
    }

    public void rightWall2Ball() {
        shooter.dropIntake();
        shooter.autoShoot(0.35);
        drive.driveToTargetBwd(77751.0); // this goes from the wall to pushing the red ball out of the way
        drive.turnLeft(85.0); // turns towards blue ball
        drive.driveToTargetFwdwIntake(-50000.0); // this drives towards the blue ball while intaking
        shooter.autoIntake(0.6);
        drive.turnRightwIntake(100); // turns right back towards goal
        drive.driveToTargetFwd(-92000.0); // drives towards goal
        shooter.kickDownAuto(); // kicks ball down
        shooter.autoShoot(0.35); // shoots
    }

    public void rightWall1Ball() {
        shooter.dropIntake();
        shooter.autoShoot(0.45);
        drive.driveToTargetBwd(98186.0); // this goes from the wall to pushing the red ball out of the way
        drive.turnLeft(75.0); // turns left towards blue ball
    }
}
