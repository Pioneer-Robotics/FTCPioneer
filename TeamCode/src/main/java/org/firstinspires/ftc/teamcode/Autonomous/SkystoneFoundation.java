package org.firstinspires.ftc.teamcode.Autonomous;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Helpers.bTelemetry;

@Autonomous(name = "Skystone Foundation", group = "ftcPio")
public class SkystoneFoundation extends Auto {

    @Override
    public void runOpMode() {
        StartRobot();
        robot.SetFoundationGripperState(0);

        waitForStart();

        robot.DriveByDistanceAndAngle(0, 1, 18.96);
        robot.DriveByDistanceAndAngle(-90, 1, 60.96);
        robot.DriveByDistanceAndAngle(0, 1, 18.96);
        robot.SetFoundationGripperState(1);
        robot.DriveByDistanceAndAngle(180, 1, 18.69 * 2.1);
        robot.DriveByDistanceAndAngle(90, 1, 152.4);

        StopMovement();
        StopRobot();
    }
}