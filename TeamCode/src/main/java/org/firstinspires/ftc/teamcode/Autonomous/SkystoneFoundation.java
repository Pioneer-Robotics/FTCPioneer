package org.firstinspires.ftc.teamcode.Autonomous;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Helpers.bTelemetry;
import org.firstinspires.ftc.teamcode.Robot.Robot;

@Autonomous(name = "Skystone Foundation", group = "ftcPio")
public class SkystoneFoundation extends Auto {

    @Override
    public void runOpMode() {
        StartRobot();
        robot.SetFoundationGripperState(0);

        waitForStart();

        robot.DriveByDistanceAndAngle(-90, 1, 80.96);
//        robot.DriveByDistanceAndAngle(-90, 1, 60.96);
        robot.DriveByDistanceAndAngle(0, 1, 94.96);
//        robot.DriveByDistanceAndAngle(0, 1, 79.96);
//        robot.DriveByDistanceAndAngle(0, 1, 18.96);
//        robot.DriveByDistanceAndAngle(0, 1, 60.96);
        robot.SetFoundationGripperState(1);
        robot.DriveByDistancePoorly(6, Robot.simpleDirection.LEFT, 0.2);
        robot.DriveByDistancePoorly(100, Robot.simpleDirection.BACKWARD, 1);

        //        robot.DriveByDistanceAndAngle(180, 0.1, 100);
//        robot.DriveByDistanceAndAngle(90, 1, 152.4);

        StopMovement();
        StopRobot();
    }
}