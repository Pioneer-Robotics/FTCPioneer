package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "Skystone3", group = "ftcPio")
public class SkystoneMain extends Auto {

    private double startRotation;

    ElapsedTime deltaTime = new ElapsedTime();

    @Override
    public void runOpMode() {
        startRobot();

        startRotation = robot.getRotation();

        waitForStart();

        //Grab the stone we are trying
        GrabArm(0.35, 0.35);

        if (side == FieldSide.SIDE_BLUE) {
            robot.rotateSimple(-90, 1, 5, 0.2);
        } else {
            robot.rotateSimple(90, 1, 5, 0.2);
        }

        //Extend the arm all of the way
        DepositeArm(1,0.2);
//
//        DepositeArm(0.35, 1);
//
//        robot.rotateSimple(0, 1, 5, 0.2);


        StopMovement();
        StopRobot();
    }
}
