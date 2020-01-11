package org.firstinspires.ftc.teamcode.Autonomous;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Robot.Robot;

@Autonomous(name = "Skystone Foundation Joe Blue", group = "ftcPio")
public class SkystoneFoundationJoeBlu extends Auto {

    @Override
    public void runOpMode() {
        StartRobot();
        robot.SetFoundationGripperState(0);
        waitForStart();
        //go forward 70 cm
        robot.DriveByDistanceAndAngle(0, 1, 90);
        //go left 75 cm
        robot.DriveByDistanceAndAngle(-90, 1, 75);
        //grip foundation
        robot.SetFoundationGripperState(1);
        //go left 6 more cm
        robot.DriveByDistanceAndAngle(-90, 0.2, 6);
        //pull it backwards
        robot.DriveByDistanceAndAngle(180, 0.5, 80);
        //release servos
        robot.SetFoundationGripperState(0);
        //go under bridge
        robot.DriveByDistanceAndAngle(90, 1, 60);
        StopRobot();
    }
}