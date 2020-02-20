package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Robot.RobotArm;

@Autonomous(name = "BLUE Computer Vision Stones", group = "ftcPio")
public class BlueSkystoneWithSelection extends Auto {

    public boolean endOnWall = false;
    public int skystoneState = 0;
    public double lengthOf3Stones = 100;

    private boolean stonePositionedLeft = false;
    private boolean stonePositionedCenter = false;
    private boolean stonePositionedRight = false;

    private double leftStoneBonus = 0;
    private double rightStoneBonus = 60;
    private double centerStoneBonus = 30;
    private double stoneBonusDistance;

    private double bridgeDistance = 144;

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
        if(skystoneState == 0) {stonePositionedLeft = true;}
        if(skystoneState == 1) {stonePositionedCenter = true;}
        if(skystoneState == 2) {stonePositionedRight = true;}

        deployGripper(true, 0.0117999);

        alignWithSkystone();

        collectFirstSkystone();

        driveToFoundationSideFirstTime(bridgeDistance + stoneBonusDistance);

        releaseStone();

        //grabs the next stone
        if(stonePositionedLeft){
        //Rolls back to the Skystone side quickly
        robot.driveByDistance(170, 1, bridgeDistance + stoneBonusDistance + lengthOf3Stones);

        //turns to face the front
        robot.rotatePID(0,0.5,1.0,5);
        collectStoneFoward(20, 200,20);
        }

        if(stonePositionedCenter){
            //Rolls back to the Skystone side quickly
            robot.driveByDistance(170, 1, bridgeDistance + stoneBonusDistance + lengthOf3Stones);

            //turns to face the front
            robot.rotatePID(0,0.5,1.0,5);

            //go a little further right first
            robot.driveByDistance(90,0.5,25);
            collectStoneFoward(20, 200,30);
        }

        if(stonePositionedRight){
            //Rolls back to the Skystone side quickly
            robot.driveByDistance(170, 1, bridgeDistance + stoneBonusDistance + lengthOf3Stones - 20);

            //turns to face the front
            robot.rotatePID(0,0.5,1.0,5);

            //go a little further right first
            robot.driveByDistance(90,0.5,55);
            collectStoneFoward(20, 200,20);
        }

        //go back across the bridge
        driveToFoundationSideSecondTime(bridgeDistance + lengthOf3Stones + stoneBonusDistance);

        releaseStone();

        park();

        StopMovement();
        StopRobot();
    }
    private void collectFirstSkystone(){
        if(stonePositionedLeft){
            collectStoneFoward(80, 100, 25);
        }
        if(stonePositionedCenter){
            collectStoneFoward(80, 100, 20);
        }
        if(stonePositionedRight){
            collectStoneFoward(80, 100, 25);
        }
    }

    private void releaseStone(){
        //Release the stone
        robot.arm.setGripState(RobotArm.GripState.OPEN, 0.5);
        //wait for the servo to finish
        sleep(100);
        //position the arm for another stone the arm
        robot.arm.setArmStateAsync(0.0117999, 0.3);
    }
    private void park(){
        if(stonePositionedLeft) {
            //back up into it's parking spot
            robot.driveByDistance(180, 0.5, 60);
            //really make sure we're there
            robot.driveByDistance(90, 0.5, 40);
        }
        if(stonePositionedCenter){
            //back up into it's parking spot
            robot.driveByDistance(180, 0.5, 80);
            //really make sure we're there
            robot.driveByDistance(90, 0.5, 60);
        }
        if(stonePositionedRight) {
            //back up into it's parking spot
            robot.driveByDistance(180, 0.5, 100);
            //really make sure we're there
            robot.driveByDistance(90, 0.5, 80);
        }
        resetArm();
    }

    private void alignWithSkystone(){ //TODO make this work for all possibilites
        //check where the skystone is and adjust left and right
        if(stonePositionedLeft){
            //move forward off the wall
            robot.driveByDistance(0.25, 10);
            stoneBonusDistance = leftStoneBonus;
            robot.driveByDistance(90,1.0,stoneBonusDistance);
        }
        if(stonePositionedCenter){
            stoneBonusDistance = centerStoneBonus;
            robot.driveByDistance(90,1.0,stoneBonusDistance);
        }
        if(stonePositionedRight){
            stoneBonusDistance = rightStoneBonus;
            robot.driveByDistance(90,1.0,stoneBonusDistance + 8);
            adjustHeading(0);
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

    //Collects the stone 'distanceToStone' away and then rolls back 'backwardDistance'
    private void collectStoneFoward(double distanceToStone, long servoDelayMS, double backwardsDistance) {
        //Drives forward so the arm is making contact with the stone
        driveToSkystone(distanceToStone);

        //Closes the gripper on the stone
        robot.arm.setGripState(RobotArm.GripState.CLOSED, 0.5);

        robot.arm.setArmStateAsync((/*target angle*/0.02148 + 0.0117999), 0.3);

        //Wait to ensure the gripper is closed
        sleep(servoDelayMS);

        //Drive backwards, breaking sticktion and allowing the gripper to close completely
        robot.driveByDistance(180, 0.5, backwardsDistance, 2);
    }

    private void driveToFoundationSideFirstTime(double bridgeDistance) {
        if(stonePositionedLeft || stonePositionedCenter) {
            //Rotates to face the foundation
            rotateAccurate(90);

            //Drives to foundation at a high speed
            robot.driveByDistance(0, 1, bridgeDistance, 2.1);
        }
        if(stonePositionedRight){
            //Rotates to face the foundation
            rotateAccurate(90);

            //Drives to foundation at a high speed
            robot.driveByDistance(5, 1, bridgeDistance);
        }
    }
    private void driveToFoundationSideSecondTime(double bridgeDistance) {
        if(stonePositionedLeft){
            driveToFoundationSideFirstTime(bridgeDistance);
        }
        if(stonePositionedCenter) {
            //Rotates to face the foundation
            rotateAccurate(90);

            //Drives to foundation at a high speed
            robot.driveByDistance(10, 1, bridgeDistance, 2.1);
        }
        if(stonePositionedRight){
            //Rotates to face the foundation
            rotateAccurate(90);

            //Drives to foundation at a high speed
            robot.driveByDistance(10, 1, bridgeDistance);
        }
    }
    public void driveToSkystone(double distanceFoward) {
        robot.driveByDistance(0, 0.35, distanceFoward, 2.6);
    }


    public void resetArm(){
        robot.arm.setGripState(RobotArm.GripState.IDLE, 0);
        robot.arm.setArmStateWait(0.03,0);
    }

    public void rotateFast(double angle) {
        robot.rotatePID(angle, 1, 2.5, 1);
//        robot.rotateSimple(angle, 2, 2, 0.5); //This one is a fail safe that will mostly work.
    }

    public void adjustHeading(double angle) {
        robot.rotatePID(angle, 1, 0.5, 1);
    }


    public void rotateAccurate(double angle) {
        robot.rotatePID(angle, 0.7, 3, 0.6);
    }
}
