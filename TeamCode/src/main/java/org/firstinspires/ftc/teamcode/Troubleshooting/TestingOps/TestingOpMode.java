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

//        robot.rotateSimple(90, 1, 2, 0.5);

        robot.experimentalDriveByDistance(0, 0.35, 0.05, 0, 25);
        robot.experimentalDriveByDistance(179, 1, 0.1, 0, 125);
        robot.experimentalDriveByDistanceWithRotationYeahItsPrettyCoool(0, 0.35, 0.2, 0, 90, 25);

//        GrabArm(0.3);

//        while (opModeIsActive()) {
//            telemetry.addData("Arm Spooool", robot.arm.length.getCurrentPosition());
//            telemetry.update();
//        }

//        robot.arm.setGripState(RobotArm.GripState.CLOSED, 0.5);

//        sleep(1000);

//        robot.arm.setArmStateWait(0.5, 0, 1);

        StopMovement();
        StopRobot();
    }

}
