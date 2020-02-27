package org.firstinspires.ftc.teamcode.CalibrationOps;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Autonomous.VuforiaBitmapSkystoneDetector;
import org.firstinspires.ftc.teamcode.Helpers.bMath;
import org.firstinspires.ftc.teamcode.Robot.Robot;

@Autonomous(name = "Camera Calibration", group = "Calibration")
public class CameraCalibration extends LinearOpMode {

    private Robot robot = new Robot();

    private VuforiaBitmapSkystoneDetector vuforiaBitmapSkystoneDetector = new VuforiaBitmapSkystoneDetector();

    VuforiaBitmapSkystoneDetector.SkystoneState currentEdit = VuforiaBitmapSkystoneDetector.SkystoneState.CENTER;

    ElapsedTime timeDelta = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(this, false);

        vuforiaBitmapSkystoneDetector.Start(this);

        waitForStart();

        double port = 0.3;
        double center = 0.5;
        double starboard = 0.6;
        boolean side_is_blue = false;

        while (opModeIsActive()) {

            telemetry.addData("Editing ", currentEdit.name());

            telemetry.addData("B Port", port);
            telemetry.addData("A Center", center);
            telemetry.addData("X Starboard", starboard);

            if (gamepad1.a) {
                currentEdit = VuforiaBitmapSkystoneDetector.SkystoneState.CENTER;
            }
            if (gamepad1.b) {
                currentEdit = VuforiaBitmapSkystoneDetector.SkystoneState.PORT;
            }
            if (gamepad1.x) {
                currentEdit = VuforiaBitmapSkystoneDetector.SkystoneState.STARBOARD;
            }

            if (currentEdit == VuforiaBitmapSkystoneDetector.SkystoneState.PORT) {
                port += gamepad1.right_stick_x * timeDelta.seconds() * 0.1;
            }

            if (currentEdit == VuforiaBitmapSkystoneDetector.SkystoneState.CENTER) {
                center += gamepad1.right_stick_x * timeDelta.seconds() * 0.1;
            }

            if (currentEdit == VuforiaBitmapSkystoneDetector.SkystoneState.STARBOARD) {
                starboard += gamepad1.right_stick_x * timeDelta.seconds() * 0.1;
            }
            port = bMath.Clamp(port, 0, 1);
            center = bMath.Clamp(center, 0, 1);
            starboard = bMath.Clamp(starboard, 0, 1);

            vuforiaBitmapSkystoneDetector.Update(this, side_is_blue);

            telemetry.addData("side:", side_is_blue ? "blue" : "red");

            telemetry.addData("State ", vuforiaBitmapSkystoneDetector.lastState.toString());

            telemetry.update();

            timeDelta.reset();
        }

        robot.shutdown();
    }
}