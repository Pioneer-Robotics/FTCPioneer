package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Robot.RobotArm;

@Autonomous(name = "Skystone Low Key Sabotage", group = "ftcPio")
public class SkystoneSelectionSabotage extends Auto {

    public double startRotation;

    ElapsedTime deltaTime = new ElapsedTime();

    double moveTime;

    @Override
    public void runOpMode() {
        StartRobot();

        startRotation = robot.GetRotation();

        waitForStart();


        GrabArm(0.5, 0.2);

        DepositeArm(0.5, 0.2);


        StopMovement();
        StopRobot();
    }
}
