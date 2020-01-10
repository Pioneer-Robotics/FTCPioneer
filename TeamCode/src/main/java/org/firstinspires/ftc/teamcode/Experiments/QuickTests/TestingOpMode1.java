package org.firstinspires.ftc.teamcode.Experiments.QuickTests;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Helpers.PID;
import org.firstinspires.ftc.teamcode.Robot.Robot;
import org.firstinspires.ftc.teamcode.Robot.RobotWallTrack;

@Autonomous(name = "TestingOpMode1", group = "Sensor")
public class TestingOpMode1 extends LinearOpMode {
    Robot robot = new Robot();

    public PID controller = new PID();

    public ElapsedTime deltaTime = new ElapsedTime();

    double targetRotation;

    double a;

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap, this, false);

        waitForStart();


        while (opModeIsActive()) {
            robot.SetFoundationGripperState(a);

            a += gamepad1.left_stick_x * deltaTime.seconds();
            telemetry.addData("Power", a);
            telemetry.update();
            deltaTime.reset();
        }

        robot.Stop();
    }

}
