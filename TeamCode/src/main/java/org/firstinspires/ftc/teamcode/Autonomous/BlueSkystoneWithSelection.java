package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Robot.RobotArm;

@Autonomous(name = "BLUE STONE SELECTION", group = "ftcPio")
public class BlueSkystoneWithSelection extends Auto {

    public boolean endOnWall = false;
    public int skystoneState = 0;
    public double lengthOf3Stones = 60;

    private boolean stonePositionedLeft = false;
    private boolean stonePositionedCenter = false;
    private boolean stonePositionedRight = false;

    //0 = PORT, 1 = CEN, 2=STBD

    VuforiaBitmapSkystoneDetector skystoneDetector = new VuforiaBitmapSkystoneDetector();


    @Override
    public void runOpMode() {
        startRobot();

        skystoneDetector.Start(this);

        speed_high = 0.5;
        speed_med = 0.50; //0.3
        speed_low = 0.3; //0.1


        while (!opModeIsActive()) {
            skystoneDetector.Update(this, true);
        }

        if (skystoneDetector.lastState == VuforiaBitmapSkystoneDetector.SkystoneState.PORT) {
            skystoneState = 0;
        }
        if (skystoneDetector.lastState == VuforiaBitmapSkystoneDetector.SkystoneState.CENTER) {
            skystoneState = 1;
        }
        if (skystoneDetector.lastState == VuforiaBitmapSkystoneDetector.SkystoneState.STARBOARD) {
            skystoneState = 2;
        }

        //start the real program
        waitForStart();

        //find the set up we're working with
//        if(skystoneState == 0) {stonePositionedLeft = true;}
//        if(skystoneState == 1) {stonePositionedCenter = true;}
//        if(skystoneState == 2) {stonePositionedRight = true;}

        stonePositionedLeft = true;

        deployGripper(true, 0.0117999);

        alignWithSkystone();

        collectStoneFoward(93, 200, 30);

        driveToFoundationSide(140);

        //Release the stone
        robot.arm.setGripState(RobotArm.GripState.OPEN, 0.5);
        //wait for the servo to finish
        sleep(200);

        robot.arm.setArmStateAsync(0.0117999, 0.3);

        //Rolls back to the skystone side quickly
        robot.driveByDistance(180, 1, 80 + lengthOf3Stones, 2.7);

        //turns to face the front
        rotateFast(0);

        //drives forward to get the other stone
        robot.driveByDistance(0.5, 15);

        //go back across the bridge
        driveToFoundationSide(lengthOf3Stones + 20);

        //back up into it's parking spot
        robot.driveByDistance(180,0.5, 20);

        StopMovement();
        StopRobot();
    }

    private void alignWithSkystone(){ //TODO make this work for all possibilites
        //move forward off the wall
        robot.driveByDistance(0.25, 10);
        //check where the skystone is and adjust left and right
        if(stonePositionedLeft){
            robot.driveByDistance(-90,1.0, 0); //starts basically lined up
        }
        if(stonePositionedCenter){
            robot.driveByDistance(0,0, 0); //should already be lined up with center
        }
        if(stonePositionedRight){
            robot.driveByDistance(90,1.0,30);
        }


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

    private void runDeliveryCycle(double fwdDistance, long servoDelayMS, double distanceFromStone, double endingOffset, double bridgeDistance, boolean prepForAnotherStone, long servoDelayShortMS) {

        collectStoneFoward(fwdDistance, servoDelayMS, distanceFromStone);


        driveToFoundationSide(bridgeDistance);

        //Releases the stone
        robot.arm.setGripState(RobotArm.GripState.OPEN, 0.5);


        //Waits to ensure the stone is completely detached
        sleep(servoDelayShortMS);

        //Resets rotation after speedyness
        adjustHeading(90);

        if (prepForAnotherStone) {
            robot.arm.setArmStateAsync(0.0117999, 0.3);

            //Rolls back to the skystone side quickly
            robot.driveByDistance(180, 1, bridgeDistance + endingOffset, 2.7); //TODO update to new
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

        robot.arm.setArmStateAsync((0.02148 + 0.0117999) / 2, 0.3);
        //        robot.arm.setArmStateAsync(0.02148, 0.3);

//        robot.arm.setArmStateAsync(0.02248, 0.3);

        //Wait to ensure the gripper is closed
        sleep(servoDelayMS);

        //Drive backwards, breaking sticktion and allowing the gripper to close completely
        robot.driveByDistance(180, 0.5, backwardsDistance, 2);
    }

    private void driveToFoundationSide(double bridgeDistance) {
        //Rotates to face the foundation
        rotateAccurate(90);

        //Drives to foundation at a high speed
        robot.driveByDistance(0, 1, bridgeDistance, 2.1); //TODO update to new
    }

    public void driveToSkystone(double distanceFoward) {
        robot.driveByDistance(0, 0.35, distanceFoward, 2.6);
    }

    public void rotateFast(double angle) {
        robot.rotatePID(angle, 1, 2.5, 1);
//        robot.rotateSimple(angle, 2, 2, 0.5); //This one is a fail safe that will mostly work.
    }

    public void adjustHeading(double angle) {
        robot.rotatePID(angle, 1, 0.5, 1);
    }


    public void rotateAccurate(double angle) {
        robot.rotatePID(angle, 1, 3, 0.6);
        //robot.rotateSimple(angle, 1, 0.5, 0.25); //This one is a fail safe that will mostly work.
    }
}
