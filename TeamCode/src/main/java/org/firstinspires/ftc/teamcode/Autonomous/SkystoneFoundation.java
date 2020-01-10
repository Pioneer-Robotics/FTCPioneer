package org.firstinspires.ftc.teamcode.Autonomous;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Skystone Foundation", group = "ftcPio")
public class SkystoneFoundation extends Auto {

    @Override
    public void runOpMode() {
        StartRobot();

        waitForStart();

        robot.DriveByDistanceAndAngle(0, 1, 60.96 - 42);
        robot.DriveByDistanceAndAngle(-90, 1, 60.96);
        robot.DriveByDistanceAndAngle(0, 1, 60.96);


        StopMovement();
        StopRobot();
    }
}