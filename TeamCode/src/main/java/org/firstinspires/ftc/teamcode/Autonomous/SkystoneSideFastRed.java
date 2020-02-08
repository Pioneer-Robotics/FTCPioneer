package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Robot.RobotArm;

@Autonomous(name = "Skystone Stone Fast Red", group = "ftcPio")
public class SkystoneSideFastRed extends Auto {


    public boolean endOnWall = false;

//    public double armGrabLength;
//    public double armGrabLength;

    @Override
    public void runOpMode() {
        startRobot();
        robot.arm.rotationMode = RobotArm.ArmThreadMode.Enabled;
        robot.arm.extensionMode = RobotArm.ArmThreadMode.Disabled;

        speed_high = 0.5;
        speed_med = 0.50; //0.3
        speed_low = 0.3; //0.1

        while (!opModeIsActive()) {

            if (true) {
                break;
            }

            if (gamepad1.a) {
                endOnWall = !endOnWall;
                sleep(500);
            }

            telemetry.addData("End state: ", endOnWall ? "Ending on WALL" : "Ending on BRIDGE");
            telemetry.addData("Press X to continue... ", "Press A to toggle wall state");
            telemetry.update();
        }

        waitForStart();

        //0.035 == lift
        deployGripper(true, 0.0117999);


//        int cycles = 2;
//
////        for (int stone = 1; stone <= cycles; stone++) {
////            runDeliveryCycle(stone == 1 ? 93 : 30, 1000, 35, stone * 24, 130 + (stone * 30), stone != cycles);
////        }

        runDeliveryCycle(100, 500, 35, 36, 150, true, 250);
//        runDeliveryCycle(93, 1000, 50, 24, 130 + (24), true);
        runDeliveryCycle(40, 500, 15, 48, 160, false,250);

        robot.arm.setArmStateAsync(0.02248, 0);

        robot.driveByDistance(180, 0.75, 35);

        robot.arm.setGripState(RobotArm.GripState.IDLE, 0);

        if (endOnWall) {
            robot.driveByDistance(90, 0.8, 80);
        } else {
            robot.driveByDistance(-90, 0.8, 50);
            robot.driveByDistance(0, 0.5, 10);
        }
//        160
//        193
//        217
//
//        runDeliveryCycle(93, 1000, 35, 24, 160, true);
//
//        runDeliveryCycle(45, 1000, 35, 1 * 24 + 24, 145 + 48, true);
//
//        runDeliveryCycle(45, 1000, 35, 2 * 24 + 24, 145 + (3 * 24), false);

        StopMovement();
        StopRobot();
    }

    //Deploys the gripper, enabling async will have the arm movement happen in the background without pausing the main thread.
    private void deployGripper(boolean async, double armLiftAmount) {

        //Sets the gripper to an idle state
        robot.arm.setGripState(RobotArm.GripState.CLOSED, 1);

        //Extends the arm
        //robot.arm.setArmStateWait(0, 0.65);
        robot.arm.setArmStateAsync(armLiftAmount, 0.65);

        sleep(500);

        //Deploys the gripper
        robot.arm.setGripState(RobotArm.GripState.OPEN, 0.5);

        sleep(800);

        if (async) {
            robot.arm.setArmStateAsync(armLiftAmount, 0.3);
        } else {
            robot.arm.setArmStateWait(0.03, 0.3);
        }
    }

    private void runDeliveryCycle(double fwdDistance, long servoDelayMS, double distanceFromStone, double endingOffset, double bridgeDistance, boolean moveBackToBridge, long servoDelayShortMS) {

        collectStoneFoward(fwdDistance, servoDelayMS, distanceFromStone);


        driveToFoundationSide(bridgeDistance);

        //Releases the stone
        robot.arm.setGripState(RobotArm.GripState.OPEN, 0.5);


        //Waits to ensure the stone is completely detached
        sleep(servoDelayShortMS);

        //Resets rotation after speedyness
        adjustHeading(-90);

        if (moveBackToBridge) {
            robot.arm.setArmStateAsync(0.0117999, 0.3);

            //Rolls back to the skystone side quickly
            robot.driveByDistance(180, 1, bridgeDistance + endingOffset, 2.7);
            //Rotates to face the next stone
            rotateFast(0);

            //Resets rotation
            //adjustHeading(90); see above rotation

            //Rolls back again so the bot is aligned with the next stone
            //robot.driveByDistance(180, 0.35, endingOffset); Because added to skystone side


        }
    }

    //Collects the stone 'distanceToStone' away and then rolls back 'backwardDistance'
    private void collectStoneFoward(double distanceToStone, long servoDelayMS, double backwardsDistance) {
        //Drives forward so the arm is making contact with the stone
        driveToSkystone(distanceToStone);

        //Closes the gripper on the stone
        robot.arm.setGripState(RobotArm.GripState.CLOSED, 0.5);

        robot.arm.setArmStateAsync(0.02148, 0.3);
//        robot.arm.setArmStateAsync(0.02248, 0.3);

        //Wait to ensure the gripper is closed
        sleep(servoDelayMS);

        //Drive backwards, breaking sticktion and allowing the gripper to close completely
        robot.driveByDistance(180, 0.5, backwardsDistance, 2);
    }

    private void driveToFoundationSide(double bridgeDistance) {
        //Rotates to face the foundation
        rotateAccurate(-90);

        //Drives to foundation at a high speed
        robot.driveByDistance(0, 1, bridgeDistance, 2.1);
    }

    public void driveToSkystone(double distanceFoward) {
        robot.driveByDistance(0, 0.35, distanceFoward, 2.6);
    }

    public void rotateFast(double angle) {
        robot.rotatePID(angle, 1, 2.5, 1);
//        robot.rotateSimple(angle, 2, 2, 0.5); //This one is a fail safe that will mostly work.
    }

    public void adjustHeading(double angle){
        robot.rotatePID(angle, 1,0.5,1);
    }


    public void rotateAccurate(double angle) {
        robot.rotatePID(angle, 1, 3, 0.6);
        //robot.rotateSimple(angle, 1, 0.5, 0.25); //This one is a fail safe that will mostly work.
    }
}
