package org.firstinspires.ftc.teamcode.Experiments.QuickTests;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Autonomous.Auto;
import org.firstinspires.ftc.teamcode.Helpers.PID;
import org.firstinspires.ftc.teamcode.Autonomous.Auto;
import org.firstinspires.ftc.teamcode.Robot.Robot;
import org.firstinspires.ftc.teamcode.Robot.RobotWallTrack;

@Autonomous(name = "TestingOpMode", group = "Sensor")
public class TestingOpMode extends Auto {
    @Override
    public void runOpMode() {
        StartRobot();

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
