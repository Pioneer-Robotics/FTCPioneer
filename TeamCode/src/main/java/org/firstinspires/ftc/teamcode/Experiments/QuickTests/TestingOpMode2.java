package org.firstinspires.ftc.teamcode.Experiments.QuickTests;


import android.renderscript.Double2;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Robot.Robot;
import org.firstinspires.ftc.teamcode.Robot.RobotWallTrack;

@Autonomous(name = "TestingOpMode2", group = "Sensor")
public class TestingOpMode2 extends LinearOpMode {

    Robot robot = new Robot();


    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap, this);

        waitForStart();

        while (opModeIsActive()) {
//            //-2623
//            telemetry.addData("Spool Position : ", robot.arm.length.getCurrentPosition());
//            //-5679
//            telemetry.addData("Lift Position : ", robot.arm.rotation.getCurrentPosition());

            telemetry.addData("POT Angle", robot.armPotentiometer.getVoltage() / 0.0122222222);
            telemetry.addData("POT Voltage", robot.armPotentiometer.getVoltage());
            telemetry.update();
        }


//        while (opModeIsActive()) {
//            telemetry.addData("180 : ", robot.wallTrack.sensorIDGroupPairs.get(RobotWallTrack.groupID.Group180).getWallAngle());
//            telemetry.addData("180 : ", robot.wallTrack.sensorIDGroupPairs.get(RobotWallTrack.groupID.Group180).getDistance(RobotWallTrack.SensorGroup.TripletType.Left, DistanceUnit.CM));
//            telemetry.addData("180 : ", robot.wallTrack.sensorIDGroupPairs.get(RobotWallTrack.groupID.Group180).getDistance(RobotWallTrack.SensorGroup.TripletType.Right, DistanceUnit.CM));
//            telemetry.addData("180 : ", robot.wallTrack.sensorIDGroupPairs.get(RobotWallTrack.groupID.Group180).getWallAngle());
//            telemetry.addData("270 : ", robot.wallTrack.sensorIDGroupPairs.get(RobotWallTrack.groupID.Group270).getWallAngle());
//            telemetry.addData("90 : ", robot.wallTrack.sensorIDGroupPairs.get(RobotWallTrack.groupID.Group180).getWallAngle());
//            telemetry.update();
//        }


//        robot.MoveSimple(0, 0.5);
//        sleep(2500);
//
//
//        robot.MoveSimple(90, 0.5);
//        sleep(2500);
//
//
//        robot.MoveSimple(180, 0.5);
//        sleep(2500);
//
//
//        robot.MoveSimple(270, 0.5);
//        sleep(2500);


        robot.Stop();

    }
}