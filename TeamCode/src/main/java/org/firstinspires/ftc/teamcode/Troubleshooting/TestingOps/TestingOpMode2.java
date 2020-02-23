package org.firstinspires.ftc.teamcode.Troubleshooting.TestingOps;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Robot.Robot;
import org.firstinspires.ftc.teamcode.Robot.RobotArm;

@Autonomous(name = "TestingOpMode2", group = "Trouble Shooting")
public class TestingOpMode2 extends LinearOpMode {

    private Robot robot = new Robot();


    @Override
    public void runOpMode() {
        robot.init(this, false);
        waitForStart();
        while (opModeIsActive()) {
            robot.experimentalDriveByDistance(0, 0.6, 0.3, 0.05, 0, 50, 10);
            robot.experimentalDriveByDistance(90, 0.6, 0.3, 0.05, 0, 50, 10);
            robot.experimentalDriveByDistance(180, 0.6, 0.3, 0.05, 0, 50, 10);
            robot.experimentalDriveByDistance(-90, 0.6, 0.3, 0.05, 0, 50, 10);

        }

        robot.shutdown();

    }
}