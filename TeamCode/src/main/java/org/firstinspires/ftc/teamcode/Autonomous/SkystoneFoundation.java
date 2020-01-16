package org.firstinspires.ftc.teamcode.Autonomous;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Robot.Robot;

@Autonomous(name = "Skystone Foundation", group = "ftcPio")
public class SkystoneFoundation extends Auto {

    @Override
    public void runOpMode() {
        StartRobot();
        robot.setFoundationGripperState(0);

        waitForStart();

        robot.driveByDistance(-90, 1, 80.96);
//        robot.driveByDistance(-90, 1, 60.96);
        robot.driveByDistance(0, 1, 94.96);
//        robot.driveByDistance(0, 1, 79.96);
//        robot.driveByDistance(0, 1, 18.96);
//        robot.driveByDistance(0, 1, 60.96);
        robot.setFoundationGripperState(1);
        robot.driveByDistancePoorly(6, Robot.simpleDirection.LEFT, 0.2);
        robot.driveByDistancePoorly(100, Robot.simpleDirection.BACKWARD, 1);

        //        robot.driveByDistance(180, 0.1, 100);
//        robot.driveByDistance(90, 1, 152.4);

        StopMovement();
        StopRobot();
    }
}