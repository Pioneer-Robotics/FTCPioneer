package org.firstinspires.ftc.teamcode.CalibrationOps;


import android.renderscript.Double3;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Helpers.bDataManager;
import org.firstinspires.ftc.teamcode.Robot.Robot;

@Autonomous(name = "PIDTuning", group = "Sensor")
public class
RotationPIDTuning extends LinearOpMode {

    private Robot robot = new Robot();

    private Double3 PID = new Double3();

    private ElapsedTime deltaTime = new ElapsedTime();

    private bDataManager dataManger = new bDataManager();


    private TuningMode mode = TuningMode.P;

    enum TuningMode {
        P, I, D
    }

    @Override
    public void runOpMode() {
        robot.init(this, true);
        dataManger.Start();
        deltaTime.reset();


        PID.x = dataManger.readData("PID_Testing_P", 3);
        PID.y = dataManger.readData("PID_Testing_I", 0.42);
        PID.z = dataManger.readData("PID_Testing_D", 0.22);

        waitForStart();

        while (opModeIsActive()) {
            //Use the controller to tune PID
            while (!gamepad1.x && opModeIsActive()) {
                if (gamepad1.y) {
                    mode = TuningMode.P;
                }
                if (gamepad1.a) {
                    mode = TuningMode.I;
                }
                if (gamepad1.b) {
                    mode = TuningMode.D;
                }


                if (mode == TuningMode.P) {
                    telemetry.addData("ADJUSTING P ", PID.x);
                    PID.x += gamepad1.right_stick_y * -1 * deltaTime.seconds();

                    if (gamepad1.right_bumper) {
                        PID.x = 0;
                    }

                }
                if (mode == TuningMode.I) {
                    telemetry.addData("ADJUSTING I ", PID.y);
                    PID.y += gamepad1.right_stick_y * -1 * deltaTime.seconds();

                    if (gamepad1.right_bumper) {
                        PID.y = 0;
                    }
                }
                if (mode == TuningMode.D) {
                    telemetry.addData("ADJUSTING D ", PID.z);
                    PID.z += gamepad1.right_stick_y * -1 * deltaTime.seconds();

                    if (gamepad1.right_bumper) {
                        PID.z = 0;
                    }
                }
                telemetry.addData("HOW TO : USE  Y TO TUNE P. USE A TO TUNE I. USE B TO TUNE D. HOLD X TO TEST A 90 TURN", "");


                telemetry.addData("P : ", PID.x);
                telemetry.addData("I : ", PID.y);
                telemetry.addData("D : ", PID.z);
                telemetry.addData("deltaTime : ", deltaTime.seconds());
                telemetry.update();
                deltaTime.reset();

            }

            dataManger.writeData("PID_Testing_P", PID.x);
            dataManger.writeData("PID_Testing_I", PID.y);
            dataManger.writeData("PID_Testing_D", PID.z);

            double targetRotation = 90 + robot.getRotation();
            robot.rotatePID(targetRotation, 1, 5, PID.x, PID.y, PID.z);
            robot.setPowerDouble4(0, 0, 0, 0, 0);
        }

        robot.shutdown();

    }
}