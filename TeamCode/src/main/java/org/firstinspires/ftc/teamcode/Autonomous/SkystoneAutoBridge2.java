package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Robot.Robot;

@Autonomous(name = "SkystoneBridge2", group = "ftcPio")
public class SkystoneAutoBridge2 extends Auto {

    public double startRotation;

    ElapsedTime deltaTime = new ElapsedTime();

    double startDelay = 5;

    double startDistance = 25;

    boolean inverted = false;

    @Override
    public void runOpMode() {
        StartRobot();
        waitForStart();
//        sleep(20000);
        robot.DriveByDistanceAndAngle(0, 0.5, 60);

        robot.DriveByDistanceAndAngle(-90, 1, 120);

        robot.DriveByDistanceAndAngle(0, 0.5, 90);
//        robot.DriveByDistanceAndAngle(95, 1, 75);
        StopMovement();
        StopRobot();
    }
}
