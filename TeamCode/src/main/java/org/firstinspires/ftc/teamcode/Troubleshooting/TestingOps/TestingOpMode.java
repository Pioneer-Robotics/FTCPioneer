package org.firstinspires.ftc.teamcode.Troubleshooting.TestingOps;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Autonomous.Auto;
import org.firstinspires.ftc.teamcode.Helpers.bTelemetry;

@Autonomous(name = "TestingOpMode", group = "Trouble Shooting")
public class TestingOpMode extends Auto {


    @Override
    public void runOpMode() {
        startRobot();

        waitForStart();

//        robot.rotateSimple(90, 1, 2, 0.5);

        bTelemetry.print("Driving by distance 0");
        robot.experimentalDriveByDistance(0, 0.35, 0.15, 0, 50);

        bTelemetry.print("Driving by distance 180");
//        robot.experimentalDriveByDistance(179, 0.8, 0.1, 0, 125);

        bTelemetry.print("Driving by wierdness 0");
        robot.experimentalDriveByDistanceWithRotationYeahItsPrettyCo0o0oOo0OoO0Oool(0, 0.35, 0.2, 0, 90, 50);

        bTelemetry.print("Spinning");
        robot.rotatePID(0, 1, 5);

        bTelemetry.print("error of " + robot.getRotation());

        sleep(1000);

        StopMovement();
        StopRobot();
    }

}
