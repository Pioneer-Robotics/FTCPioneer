package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Robot.RobotArm;

@Autonomous(name = "Skystone Stone Fast", group = "ftcPio")
public class SkystoneSideFast extends Auto {

    @Override
    public void runOpMode() {
        startRobot();

        speed_high = 0.5;
        speed_med = 0.30;
        speed_low = 0.1;

        waitForStart();

        DeployGripper(true);


        int cycles = 3;

        for (int stone = 1; stone < cycles; stone++) {
            RunDeliveryCycle(stone == 1 ? 93 : 45, 1000, 35, stone * 24, 130 + (stone * 30), stone != cycles);
        }

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
        robot.arm.setArmStateWait(0, 0.5);

        //Deploys the gripper
        robot.arm.SetGripState(RobotArm.GripState.OPEN, 0.5);

        if (async) {
            robot.arm.setArmStateAsync(0, 0.3);
        } else {
            robot.arm.setArmStateWait(0, 0.3);
        }
    }

    private void RunDeliveryCycle(double fwdDistance, long servoDelayMS, double distanceFromStone, double endingOffset, double bridgeDistance, boolean moveBackToBridge) {

        driveToSkystone(fwdDistance);

        robot.arm.SetGripState(RobotArm.GripState.CLOSED, 0.5);

        sleep(servoDelayMS);

        robot.driveByDistance(180, 0.5, distanceFromStone);

        //Rotates to face foundation
        RotateFast(90);

        //Drives to foundation
        robot.driveByDistance(0, 1, bridgeDistance);

        //Drop stone
        robot.arm.SetGripState(RobotArm.GripState.OPEN, 0.5);

        sleep(servoDelayMS);

        //Resets rotation after speedyness
        RotateFast(90);

        if (moveBackToBridge) {

            //Wait to make sure the stone is dropped
            sleep(servoDelayMS / 2);

            //Rolls back to the skystone side
            robot.driveByDistance(180, 1, bridgeDistance);

            //Rerests rotation
            RotateFast(90);

            //Rolls back again the required distance
            robot.driveByDistance(180, 0.35, endingOffset);

            RotateFast(0);
        }
    }

    public void driveToSkystone(double distanceFoward) {
        robot.driveByDistance(0, 0.35, distanceFoward);
    }

    public void RotateFast(double angle) {
        robot.rotateSimple(angle, speed_high, 2, 0.5);
    }


    public void RotationPrecise(double angle) {
        robot.rotateSimple(angle, speed_high, 2, 0.5);
    }
}
