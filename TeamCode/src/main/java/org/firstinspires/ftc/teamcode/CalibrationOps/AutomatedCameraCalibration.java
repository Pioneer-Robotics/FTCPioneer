package org.firstinspires.ftc.teamcode.CalibrationOps;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Autonomous.VuforiaBitmapSkystoneDetector;
import org.firstinspires.ftc.teamcode.Helpers.bMath;
import org.firstinspires.ftc.teamcode.Robot.Robot;

@Autonomous(name = "[WIP] Automatic Camera Calibration", group = "Calibration")
public class AutomatedCameraCalibration extends LinearOpMode {

    private Robot robot = new Robot();

    private VuforiaBitmapSkystoneDetector vuforiaBitmapSkystoneDetector = new VuforiaBitmapSkystoneDetector();

    VuforiaBitmapSkystoneDetector.SkystoneState currentEdit = VuforiaBitmapSkystoneDetector.SkystoneState.CENTER;

    ElapsedTime timeDelta = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(this, false);

        vuforiaBitmapSkystoneDetector.Start(this);

        waitForStart();

        int offset = 0;
        double port = 0.3;
        double center = 0.5;
        double starboard = 0.6;

        telemetry.addData("Place the skystone in the PORT position", " Press X to continue");
        telemetry.update();

        waitForX();


        while (opModeIsActive()) {
            vuforiaBitmapSkystoneDetector.Update(this, port, center, starboard);
//            vuforiaBitmapSkystoneDetector
        }

        robot.shutdown();
    }

    void waitForX() {
        while (!gamepad1.x) {
        }
    }
}