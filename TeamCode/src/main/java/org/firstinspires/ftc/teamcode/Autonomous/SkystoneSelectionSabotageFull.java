package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Robot.RobotArm;

@Autonomous(name = "Skystone Low Key Sabotage Full", group = "ftcPio")
public class SkystoneSelectionSabotageFull extends Auto {

    /// Here is an example of a refactored version of SkystoneSelectionSabotageFull.java
    /// I'm adding this here as a reference so that you can see why refactoring is helpful.
    /// All of the code commented with /// is explaining why or asking questions and is not necessary to the file itself.


    /// This code now has levels of complexity. This is important both for when reading code and for debugging code.
    /// It doesn't change the operation of the code, but is essential to the development process which includes change orders and debugging.
    /// One very important concept in coding to grasp is that good code is written for humans, not for computers. Computers will do what they're told no matter how complicated it is.
    /// Humans are needed to understand the code so that they can debug and change it without inadvertently causing bugs and generally making mistakes.

    /// Eventually, the levels of the code should be separated into different files. This allows the coder to only see the level of abstraction that they need to work on, which minimizes coder error.



    // state variables
    private double startRotation; /// Is this necessary? It doesn't appear to be used currently other than being set in setupRobotForAutonomous()


    /// This is the highest level of code abstraction is simple to read and has variable that might need to be tinkered with.
    /// This is where you want to be debugging and changing things because it's simple to understand and the least amount of possible mistakes can be introduced by coder error.

    @Override
    public void runOpMode() {
        // Setup
        setupRobotForAutonomous();
        deployGripperToGrabBlocksPosition();
        extendArmToOptimalPosition();

        // Variables that may need to be altered
        double initialFwdDistance = 93;
        long servoDelayMS = 1000;
        double bridgeDistance = 145;
        double distanceFromStone = 35;
        double initialEndingOffset = 24;
        int numBlocksToAttempt = 2;

        // Grab Blocks
        // initial sequence
        RunDeliveryCycle(initialFwdDistance, servoDelayMS, distanceFromStone, initialEndingOffset, bridgeDistance);

        // All additional sequences
        for (int i = 0; i < numBlocksToAttempt; i++) {
            double additionalOffset = (i * initialEndingOffset);
            double totalIterationOffset = additionalOffset + initialEndingOffset;
            RunDeliveryCycle(0, servoDelayMS, distanceFromStone, totalIterationOffset, bridgeDistance);
        }

        // Stop
        StopMovement();
        StopRobot();
    }

    //----------------------------------------------------------------------------------------------

    /// In this next level of abstraction the idea of running the delivery cycle is broken down into easily understood parts that can be tested individually as issues arise.

    // Run Delivery Cycle
    private void RunDeliveryCycle(double fwdDistance, long servoDelayMS, double distanceFromStone, double endingOffset, double bridgeDistance) {

        moveForwardToBlocks(fwdDistance);

        grabBlock(servoDelayMS, distanceFromStone);

        moveUnderBridge(bridgeDistance);

        dropOffStone(servoDelayMS);

        repositionRobotForNextStone(bridgeDistance, endingOffset);
    }


    //----------------------------------------------------------------------------------------------

    /// These helper methods are the next level of abstraction. They should rarely need to be changed.
    /// If they do need to be changed the level of abstraction should shield coder mistakes since the methods are walled off from other methods.

    /// This should only have magic numbers that will not change. I'm not sure if that is the case here, so this may need more refactoring.

    // Setup Helpers
    private void setupRobotForAutonomous() {
        startRobot();

        startRotation = robot.getRotation();

        /// Do these values need to be set at a different abstraction?
        speed_high = 0.5;
        speed_med = 0.30;
        speed_low = 0.1;

        waitForStart();
    }

    private void deployGripperToGrabBlocksPosition() {
        //Deploy gripper
        robot.arm.setGripState(RobotArm.GripState.CLOSED, 1);

        robot.arm.setArmStateWait(0, 0.5);

        robot.arm.setGripState(RobotArm.GripState.OPEN, 0.5);

        sleep(250); /// Does the code need to sleep here?
    }

    private void extendArmToOptimalPosition() {

        robot.arm.setArmStateWait(0, 0.3);
    }

    //----------------------------------------------------------------------------------------------

    // Run Delivery Cycle - Helpers
    private void moveForwardToBlocks(double fwdDistance) {
        robot.driveByDistance(0, 0.35, fwdDistance);

        sleep(500); /// should all sleep values be set in a centralized place so that they can be edited, or is this never going to change?
    }

    private void grabBlock(long servoDelayMS, double distanceFromStone) {
        robot.arm.SetGripState(RobotArm.GripState.CLOSED, 0.5);

        sleep(servoDelayMS);

        robot.driveByDistance(180, 0.5, distanceFromStone);
    }

    private void moveUnderBridge(double bridgeDistance) {
        //Rotates to face foundation
        robot.rotatePID(90, 1, 2);

        //Drives to foundation
        robot.driveByDistance(0, 1, bridgeDistance);
    }

    private void dropOffStone(long servoDelayMS) {
        robot.arm.setArmStateWait(0.2, 0.8);

        //Drop stone
        robot.arm.setGripState(RobotArm.GripState.OPEN, 0.5);

        sleep(servoDelayMS);
    }

    private void repositionRobotForNextStone(double bridgeDistance, double endingOffset) {
        robot.driveByDistance(180, 1, bridgeDistance);

        robot.rotatePID(90, 1, 2);

        robot.driveByDistance(180, 0.35, endingOffset);

        robot.rotatePID(0, 1, 3);

        robot.driveByDistance(180, 1, 90);

        robot.rotatePID(0, 1, 2);
    }

}
