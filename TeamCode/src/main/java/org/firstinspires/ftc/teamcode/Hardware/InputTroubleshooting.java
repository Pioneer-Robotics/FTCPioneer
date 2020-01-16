package org.firstinspires.ftc.teamcode.Hardware;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Robot.Robot;
import org.firstinspires.ftc.teamcode.Robot.RobotWallTrack;

@Autonomous(name = "Wheel Calibration", group = "Calibration")
public class InputTroubleshooting extends LinearOpMode {

    private Robot robot = new Robot();

    private ElapsedTime deltaTime = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        robot.initCalibration(hardwareMap, this);
        start();
        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("Rotation               : ", robot.getRotation());
            telemetry.addData("============================", robot.getDistance(RobotWallTrack.groupID.Group270, DistanceUnit.CM));
            telemetry.addData("Distance Sensor 90   : ", robot.getDistance(RobotWallTrack.groupID.Group90, DistanceUnit.CM));
            telemetry.addData("Distance Sensor 180  : ", robot.getDistance(RobotWallTrack.groupID.Group180, DistanceUnit.CM));
            telemetry.addData("Distance Sensor 270  : ", robot.getDistance(RobotWallTrack.groupID.Group270, DistanceUnit.CM));
            telemetry.addData("Arm Rotation", robot.arm.rotation.getCurrentPosition());
            telemetry.addData("Arm Spool", robot.arm.length.getCurrentPosition());

            telemetry.update();
        }

        robot.shutdown();
    }
}