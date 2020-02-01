package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Robot.RobotArm;

@Autonomous(name = "Skystone Stone Fast", group = "ftcPio")
public class SkystoneSideFast extends Auto {

    @Override
    public void runOpMode() {
        startRobot();
        robot.arm.rotationMode = RobotArm.ArmRotationMode.Threaded;

        speed_high = 0.5;
        speed_med = 0.30;
        speed_low = 0.1;

        waitForStart();

        DeployGripper(true);


//        int cycles = 2;
//
////        for (int stone = 1; stone <= cycles; stone++) {
////            RunDeliveryCycle(stone == 1 ? 93 : 30, 1000, 35, stone * 24, 130 + (stone * 30), stone != cycles);
////        }

        RunDeliveryCycle(93, 1000, 50, 24, 130 + (24), true);
        RunDeliveryCycle(45, 1000, 35, 24, 130 + (48), false);


        robot.driveByDistance(180, 0.75, 22.86);
        robot.driveByDistance(90, 0.8, 50);

//        160
//        193
//        217
//
//        RunDeliveryCycle(93, 1000, 35, 24, 160, true);
//
//        RunDeliveryCycle(45, 1000, 35, 1 * 24 + 24, 145 + 48, true);
//
//        RunDeliveryCycle(45, 1000, 35, 2 * 24 + 24, 145 + (3 * 24), false);

        StopMovement();
        StopRobot();
    }

    //Deploys the gripper, enabling async will have the arm movement happen in the background without pausing the main thread.
    private void DeployGripper(boolean async) {

        //Sets the gripper to an idle state
        robot.arm.SetGripState(RobotArm.GripState.CLOSED, 1);

        //Extends the arm
        robot.arm.setArmStateWait(0, 0.65);

        sleep(500);

        //Deploys the gripper
        robot.arm.SetGripState(RobotArm.GripState.OPEN, 0.5);

        sleep(800);

        if (async) {
            robot.arm.setArmStateAsync(0.03, 0.3);
        } else {
            robot.arm.setArmStateWait(0.03, 0.3);
        }
    }

    private void RunDeliveryCycle(double fwdDistance, long servoDelayMS, double distanceFromStone, double endingOffset, double bridgeDistance, boolean moveBackToBridge) {

        driveToSkystone(fwdDistance);

        robot.arm.SetGripState(RobotArm.GripState.CLOSED, 0.5);

        sleep(servoDelayMS);

        robot.driveByDistance(180, 0.5, distanceFromStone, 2);

        //Rotates to face foundation
        RotateAccurate(90);

        //Drives to foundation
        robot.driveByDistance(0, 1, bridgeDistance, 2.1);

        //Drop stone
        robot.arm.SetGripState(RobotArm.GripState.OPEN, 0.5);

        sleep(servoDelayMS);

        //Resets rotation after speedyness
        RotateFast(90);

        if (moveBackToBridge) {

            //Wait to make sure the stone is dropped
            sleep(servoDelayMS / 2);

            //Rolls back to the skystone side
            robot.driveByDistance(180, 1, bridgeDistance, 2.4);

            //Rerests rotation
            RotateFast(90);

            //Rolls back again the required distance
            robot.driveByDistance(180, 0.35, endingOffset);

            RotateFast(0);
        }
    }

    public void driveToSkystone(double distanceFoward) {
        robot.driveByDistance(0, 0.35, distanceFoward, 2.6);
    }

    public void RotateFast(double angle) {
        robot.rotatePID(angle, 2, 2);
//        robot.rotateSimple(angle, 2, 2, 0.5);
    }


    public void RotateAccurate(double angle) {
        robot.rotatePID(angle, 2, 2);

        //        robot.rotateSimple(angle, 1, 0.5, 0.25);
    }
}
