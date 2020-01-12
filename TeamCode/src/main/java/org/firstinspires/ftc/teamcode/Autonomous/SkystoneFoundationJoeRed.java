package org.firstinspires.ftc.teamcode.Autonomous;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Robot.Robot;

@Autonomous(name = "Skystone Foundation Joe Red", group = "ftcPio")
public class SkystoneFoundationJoeRed extends Auto {

    private boolean endOnWall = true;

    @Override
    public void runOpMode() {
        StartRobot();
        robot.SetFoundationGripperState(0);
        waitForStart();
        //go forward 70 cm
        robot.DriveByDistanceAndAngle(0, 0.5, 90);
        //go right 75 cm, (check)
        robot.DriveByDistanceAndAngle(80, 0.5, 60);
        //go forward 20 cm to correct
        robot.DriveByDistanceAndAngle(0, 0.5, 20);
        //grip foundation
        robot.SetFoundationGripperState(1);
        //delay
        sleep(2000);
        robot.DriveByDistanceAndAngle(180, 0.5, 100);

        robot.RotatePID(-35, 1, 3);
        robot.SetFoundationGripperState(0);
        sleep(2000);
        robot.RotatePID(0, 1, 3);

        robot.DriveByDistanceAndAngle(180, 0.5, 50);
//        robot.DriveByDistanceAndAngle(180, 0.5, 120);

        //pull it backwards
//        robot.DriveByDistanceAndAngle(180, 0.5, 120);
        //release servos
//        robot.SetFoundationGripperState(0);
        //go under bridge (check)
        robot.DriveByDistanceAndAngle(-100, 1, 250);
        //pull into the wall, to make sure you're there. Or don't (if/else statement)
        if (endOnWall) {
            robot.DriveByDistanceAndAngle(180, 1, 5);
        } else {
            robot.RotatePID(0, 1, 2);
            robot.DriveByDistanceAndAngle(0, 1, 150);
        }
        StopMovement();
        StopRobot();
    }
}