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
        robot.experimentalDriveByDistance(0, 0.35, 0.35, 0.05, 0, 50, 5);


        sleep(1000);

        StopMovement();
        StopRobot();
    }

}
