package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Robot.RobotArm;

@Autonomous(name = "Skystone Low Key Sabotage Full", group = "ftcPio")
public class SkystoneSelectionSabotageFull extends Auto {

    private double startRotation;

    @Override
    public void runOpMode() {
        startRobot();

        startRotation = robot.getRotation();

        speed_high = 0.5;
        speed_med = 0.30;
        speed_low = 0.1;

        waitForStart();

        //Deploy gripper
        robot.arm.setGripState(RobotArm.GripState.CLOSED, 1);

        //MIN IS .17
//        robot.arm.setArmStateWait(0, 0, 1);

        robot.arm.setArmStateWait(0, 0.5);

        robot.arm.setGripState(RobotArm.GripState.OPEN, 0.5);

        sleep(250);

        robot.arm.setArmStateWait(0, 0.3);
        RunDeliveryCycle(93, 1000, 35, 24, 145);
///
        for (int i = 0; i < 2; i++) {
            RunDeliveryCycle(0, 1000, 35, i * 24 + 24, 145);
        }
        StopMovement();
        StopRobot();
    }

    private void RunDeliveryCycle(double fwdDistance, long servoDelayMS, double distanceFromStone, double endingOffset, double bridgeDistance) {


        robot.driveByDistance(0, 0.35, fwdDistance);

        sleep(500);

        robot.arm.setGripState(RobotArm.GripState.CLOSED, 0.5);

        sleep(servoDelayMS);

        robot.driveByDistance(180, 0.5, distanceFromStone);

//        robot.rotatePIDRelative(-90, 1, 3);
        //Rotates to face foundation
        robot.rotatePID(90, 1, 2);

        //Drives to foundation
        robot.driveByDistance(0, 1, bridgeDistance);

        robot.arm.setArmStateWait(0.2, 0.8);

        //Drop stone
        robot.arm.setGripState(RobotArm.GripState.OPEN, 0.5);

        sleep(servoDelayMS);

        robot.driveByDistance(180, 1, bridgeDistance);

        robot.rotatePID(90, 1, 2);

        robot.driveByDistance(180, 0.35, endingOffset);

        robot.rotatePID(0, 1, 3);

        robot.driveByDistance(180, 1, 90);

        robot.rotatePID(0, 1, 2);
    }

}
