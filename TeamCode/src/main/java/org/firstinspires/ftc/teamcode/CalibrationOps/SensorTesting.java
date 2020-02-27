package org.firstinspires.ftc.teamcode.CalibrationOps;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Robot.Robot;
import org.firstinspires.ftc.teamcode.Robot.RobotWallTrack;

//As of 12.3.19 0708 this serves to test the timing of input methods
//As of 9.15.19.0706 this serves only to test sensor input
@Autonomous(name = "Sensors", group = "Sensor")
public class SensorTesting extends LinearOpMode {
    Robot robot = new Robot();

    ElapsedTime deltaTime = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(this, true);

        waitForStart();


        //Loopy loop loop that loops
        while (opModeIsActive()) {
            deltaTime.reset();

            telemetry.addData("Active threads", Thread.activeCount());

            deltaTime.reset();
            telemetry.addData("Front Left", robot.driveManager.frontLeft.getCurrentPosition());
            telemetry.addData("Time (MS) ", deltaTime.milliseconds());

            deltaTime.reset();
            telemetry.addData("Front Right", robot.driveManager.frontRight.getCurrentPosition());
            telemetry.addData("Time (MS) ", deltaTime.milliseconds());

            deltaTime.reset();
            telemetry.addData("Back Right", robot.driveManager.backRight.getCurrentPosition());
            telemetry.addData("Time (MS) ", deltaTime.milliseconds());

            deltaTime.reset();
            telemetry.addData("Back Left", robot.driveManager.backLeft.getCurrentPosition());
            telemetry.addData("Time (MS) ", deltaTime.milliseconds());


            telemetry.update();
        }

    }

}
