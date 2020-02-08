package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

//@Autonomous(name = "SkystoneBridgeSimple", group = "ftcPio")
public class SkystoneAutoBridgeSimple extends Auto {

    @Override
    public void runOpMode() {
        startRobot();
        waitForStart();
//        sleep(20000);
        robot.driveByDistance(0, 0.5, 60);

        robot.driveByDistance(-90, 1, 120);

        robot.driveByDistance(0, 0.5, 90);
//        robot.driveByDistance(95, 1, 75);
        StopMovement();
        StopRobot();
    }
}
