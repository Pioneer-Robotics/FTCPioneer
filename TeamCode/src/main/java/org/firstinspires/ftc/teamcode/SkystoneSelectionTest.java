package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "Skystone2", group = "ftcPio")
public class SkystoneSelectionTest extends Auto {

    public double startRotation;

    public VuforiaSkystoneDetector detector;

    ElapsedTime deltaTime = new ElapsedTime();

    @Override
    public void runOpMode() {
        StartRobot();

        startRotation = robot.GetRotation();

        detector.Start(this);

        while (!opModeIsActive()) {
            detector.Update(this);
        }

        waitForStart();

        GrabArm(0.35, 0.35);

//        if (side == FieldSide.SIDE_BLUE) {
//            robot.RotateSimple(-90, 1, 5, 0.2);
////            robot.RotatePID(-90, 1, 100);
//        } else {
//            robot.RotateSimple(90, 1, 5, 0.2);
//
////            robot.RotatePID(90, 1, 100);
//        }
//
//        DepositeArm(0.35, 1);
//
//        robot.RotateSimple(0, 1, 5, 0.2);

        detector.Stop();
        StopMovement();
        StopRobot();
    }
}
