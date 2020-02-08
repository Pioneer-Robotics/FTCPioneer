package org.firstinspires.ftc.teamcode.Troubleshooting.TestingOps;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.teamcode.Autonomous.Auto;
import org.firstinspires.ftc.teamcode.Helpers.bTelemetry;
import org.firstinspires.ftc.teamcode.Robot.RobotArm;

@Autonomous(name = "TestingOpMode", group = "Trouble Shooting")
public class TestingOpMode extends Auto {


    @Override
    public void runOpMode() {
        startRobot();

        waitForStart();

//        robot.rotateSimple(90, 1, 2, 0.5);

        bTelemetry.print("Driving by distance 0");
        robot.experimentalDriveByDistance(0, 0.35, 0.05, 0, 25);

        bTelemetry.print("Driving by distance 180");
        robot.experimentalDriveByDistance(179, 1, 0.1, 0, 125);

        bTelemetry.print("Driving by wierdness 0");
        robot.experimentalDriveByDistanceWithRotationYeahItsPrettyCoool(0, 0.35, 0.2, 0, 90, 50);

        bTelemetry.print("Spinning");
        robot.rotatePID(0, 1, 5);

        bTelemetry.print("error of " + robot.getRotation());

        sleep(1000);

        StopMovement();
        StopRobot();
    }

}
