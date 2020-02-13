package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

//@Autonomous(name = "SkystoneAutonomous", group = "ftcPio")
public class SkystoneSelectionTest extends Auto {

    private double startRotation;

    private VuforiaSkystoneDetector detector = new VuforiaSkystoneDetector();

    private ElapsedTime deltaTime = new ElapsedTime();

    private double moveTime;

    @Override
    public void runOpMode() {
        startRobot();

        startRotation = robot.getRotation();

        detector.Start(this);

        while (!opModeIsActive()) {
        }


        waitForStart();

        robot.driveByDistance(0, 0.35, 70);

        while (opModeIsActive()) {
            detector.Update(this);
            telemetry.addData("Selection ", detector.lastState.toString());
            telemetry.update();
        }
        //
//        if (detector.lastState == VuforiaSkystoneDetector.SkystoneState.CENTER) {
//
//        }
//        if (detector.lastState == VuforiaSkystoneDetector.SkystoneState.PORT) {
//            while (opModeIsActive()) {
//                deltaTime.reset();
//
//                robot.moveComplex(-90, speed_low, startRotation - robot.getRotation(), 0);
//
//                moveTime += deltaTime.time();
//
//                if (moveTime > 1) {
//                    break;
//                }
//            }
//        }
//        if (detector.lastState == VuforiaSkystoneDetector.SkystoneState.STARBOARD) {
//            while (opModeIsActive()) {
//                deltaTime.reset();
//
//                robot.moveComplex(90, speed_low, startRotation - robot.getRotation(), 0);
//
//                moveTime += deltaTime.time();
//
//                if (moveTime > 1) {
//                    break;
//                }
//            }
//        }
//
//
//        GrabArm(0.35, 0.35);
//
//
//        if (side == FieldSide.SIDE_BLUE) {
//            robot.rotateSimple(-90, 1, 5, 0.2);
//        } else {
//            robot.rotateSimple(90, 1, 5, 0.2);
//        }
//
//        DepositeArm(0.35, 1);
//
//        robot.rotateSimple(0, 1, 5, 0.2);
//
//        //Move to the next skystone, all stones have the same offset this will need big calibration


        StopMovement();
        StopRobot();
    }
}
