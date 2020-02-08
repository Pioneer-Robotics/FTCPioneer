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

        robot.arm.rotationMode = RobotArm.ArmThreadMode.Enabled;
//        robot.arm.rotationMode = RobotArm.ArmRotationMode.Disabled;

//        robot.arm.rotation.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        robot.arm.rotation.setPower(1);
//        sleep(2500);
//        robot.arm.rotation.setPower(0);

//        robot.arm.setArmStateAsync(0.45, 0);
//        sleep(10000);

//        robot.arm.setArmStateAsync(0.03, 0);
//        sleep(10000);
//
//        robot.arm.setArmStateAsync(0.035, 0);
//        sleep(10000);

//        robot.arm.setArmStateWait(0.02, 0.65);
//        robot.arm.setArmStateWait(0.01, 0.65);
//        robot.arm.setArmStateWait(0.03, 0.65);
//        robot.arm.setArmStateWait(0.03, 0.65);

        robot.shutdown();

    }
}