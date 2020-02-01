package org.firstinspires.ftc.teamcode.Troubleshooting.TestingOps;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.teamcode.Autonomous.Auto;
import org.firstinspires.ftc.teamcode.Robot.RobotArm;

@Autonomous(name = "TestingOpMode", group = "Trouble Shooting")
public class TestingOpMode extends Auto {


    @Override
    public void runOpMode() {
        startRobot();

        waitForStart();

        robot.rotateSimple(90, 1, 2, 0.5);

//        GrabArm(0.3);

//        while (opModeIsActive()) {
//            telemetry.addData("Arm Spooool", robot.arm.length.getCurrentPosition());
//            telemetry.update();
//        }

//        robot.arm.SetGripState(RobotArm.GripState.CLOSED, 0.5);

//        sleep(1000);

//        robot.arm.setArmStateWait(0.5, 0, 1);

        StopMovement();
        StopRobot();
    }

}
