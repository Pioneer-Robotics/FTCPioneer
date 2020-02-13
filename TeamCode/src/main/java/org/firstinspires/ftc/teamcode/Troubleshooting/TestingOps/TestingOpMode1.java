package org.firstinspires.ftc.teamcode.Troubleshooting.TestingOps;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Helpers.PID;
import org.firstinspires.ftc.teamcode.Robot.Robot;

@Autonomous(name = "TestingOpMode1", group = "Trouble Shooting")
public class TestingOpMode1 extends LinearOpMode {
    private Robot robot = new Robot();

    public PID controller = new PID();

    public ElapsedTime deltaTime = new ElapsedTime();

    double targetRotation;

    private double a;

    @Override
    public void runOpMode() {
        robot.init(this, false);

        waitForStart();


        while (opModeIsActive()) {
            robot.setFoundationGripperState(a);

            a += gamepad1.left_stick_x * deltaTime.seconds() * 100;

//            robot.arm.setArmStateWait(1,);

            telemetry.addData("Power", a);
            telemetry.update();
            deltaTime.reset();
        }

        robot.shutdown();
    }

}
