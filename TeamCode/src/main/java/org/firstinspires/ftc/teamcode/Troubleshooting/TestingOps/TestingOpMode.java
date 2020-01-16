package org.firstinspires.ftc.teamcode.Troubleshooting.TestingOps;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Autonomous.Auto;

@Autonomous(name = "TestingOpMode", group = "Sensor")
public class TestingOpMode extends Auto {
    @Override
    public void runOpMode() {
        startRobot();

        waitForStart();

        GrabArm(0.25, 0.25);

        while (opModeIsActive()) {
            telemetry.addData("ARM POSITION ROTATION", robot.arm.rotation.getCurrentPosition());
            telemetry.update();
        }

        StopMovement();
        StopRobot();
    }

}
