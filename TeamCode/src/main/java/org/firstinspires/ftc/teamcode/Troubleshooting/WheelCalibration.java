package org.firstinspires.ftc.teamcode.Troubleshooting;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Robot.Robot;

@Autonomous(name = "Wheel Calibration yay", group = "Calibration")
public class WheelCalibration extends LinearOpMode {

    private Robot robot = new Robot();

    private ElapsedTime deltaTime = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        robot.initCalibration(hardwareMap, this);

        waitForStart();

        double calTime = getRuntime();


        while (opModeIsActive()) {
            telemetry.addData("Calibration completed in " + calTime + " seconds, please press start to exit.", "");
            telemetry.update();

        }

        robot.shutdown();
    }
}