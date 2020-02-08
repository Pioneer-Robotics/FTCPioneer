package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "SkystoneBridge", group = "ftcPio")
public class SkystoneAutoBridge extends Auto {

    private double startRotation;

    private ElapsedTime deltaTime = new ElapsedTime();

    private double startDelay = 5;

    private double startDistance = 25;

    private boolean inverted = false;

    @Override
    public void runOpMode() {
        startRobot();

        while (!opModeIsActive()) {
            startDelay += gamepad1.left_stick_x * deltaTime.seconds() * 5;
            startDistance += gamepad1.right_stick_x * deltaTime.seconds() * 10;
            if (gamepad1.a) {
                inverted = true;
            }
            if (gamepad1.b) {
                inverted = false;
            }
            telemetry.addData("start Delay (seconds) ", startDelay);
            telemetry.addData("Drive Distance (CM) ", startDistance);
            telemetry.addData("Drive Inverted (True == Left/Right first. False == drive fwrd first) ", inverted);
            telemetry.update();
            deltaTime.reset();
        }
        waitForStart();

        sleep((long) startDelay * 1000);

        if (inverted) {
            robot.driveByDistance(0.25, 5);


            if (side == FieldSide.SIDE_BLUE) {
                robot.moveComplex(-90, 0.75, robot.getRotation() - startRotation, 0);
            }
            if (side == FieldSide.SIDE_RED) {
                robot.moveComplex(90, 0.75, robot.getRotation() - startRotation, 0);
            }

            sleep(1500);

            robot.driveByDistance(0.25, startDistance - 5);

        } else {

//        robot.driveByDistance(0.25, 5);
            robot.driveByDistance(0.25, startDistance);


            if (side == FieldSide.SIDE_BLUE) {
                robot.moveComplex(-90, 0.75, robot.getRotation() - startRotation, 0);
            }
            if (side == FieldSide.SIDE_RED) {
                robot.moveComplex(90, 0.75, robot.getRotation() - startRotation, 0);
            }

            sleep(1500);

        }
        StopMovement();
        StopRobot();
    }
}
