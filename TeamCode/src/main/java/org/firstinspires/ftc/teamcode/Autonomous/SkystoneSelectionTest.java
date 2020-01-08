package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "Skystone2", group = "ftcPio")
public class SkystoneSelectionTest extends Auto {

    public double startRotation;

    public VuforiaSkystoneDetector detector = new VuforiaSkystoneDetector();

    ElapsedTime deltaTime = new ElapsedTime();

    double moveTime;

    @Override
    public void runOpMode() {
        StartRobot();

        startRotation = robot.GetRotation();

        detector.Start(this);

        while (!opModeIsActive()) {
            detector.Update(this);
        }


        waitForStart();

        detector.Stop();

        if (detector.lastState == VuforiaSkystoneDetector.SkystoneState.CENTER) {
        }
        if (detector.lastState == VuforiaSkystoneDetector.SkystoneState.PORT) {
            while (opModeIsActive()) {
                deltaTime.reset();

                robot.MoveComplex(-90, speed_low, startRotation - robot.GetRotation());

                moveTime += deltaTime.time();

                if (moveTime > 1) {
                    break;
                }
            }
        }
        if (detector.lastState == VuforiaSkystoneDetector.SkystoneState.STARBOARD) {
            while (opModeIsActive()) {
                deltaTime.reset();

                robot.MoveComplex(90, speed_low, startRotation - robot.GetRotation());

                moveTime += deltaTime.time();

                if (moveTime > 1) {
                    break;
                }
            }
        }


        GrabArm(0.35, 0.35);


        if (side == FieldSide.SIDE_BLUE) {
            robot.RotateSimple(-90, 1, 5, 0.2);
        } else {
            robot.RotateSimple(90, 1, 5, 0.2);
        }

        DepositeArm(0.35, 1);

        robot.RotateSimple(0, 1, 5, 0.2);

        //Move to the next skystone, all stones have the same offset this will need big calibration




        StopMovement();
        StopRobot();
    }
}
