package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Helpers.PID;
import org.firstinspires.ftc.teamcode.Robot.Robot;

public class Auto extends LinearOpMode {

    public Robot robot = new Robot();

    public double speed_low = 0.15;
    public double speed_med = 0.35;
    public double speed_high = 0.85;

    public PID walltrackingController = new PID();

    //What side we are playing on, based on the bridge colors

    public FieldSide side;

    public enum FieldSide {
        SIDE_BLUE,
        SIDE_RED
    }

    @Override
    public void runOpMode() {

    }

    public void startRobot() {

        print("Status: Initiating robot.");

        //init the bot hardware! This sets up the static references for the bot as well so make sure to run this before any other code
        robot.init(this, false);

        print("Status: Initiating all jobs.");

    }

    //Stops the robot by setting power of all motors to 0
    public void StopRobot() {
        robot.setPowerDouble4(0, 0, 0, 0, 0);
        robot.shutdown();
    }

    /**
     * @param moveSpeed             How fast we will be moving side to side (lower speeds recommended)
     * @param wallDistance          How far away from the rear wall we should try to be, in CM
     * @param correctionCoefficient How sensitive we are to the position of the skystone, lower values recommended
     * @param lockTime              How long we must be within lockThreshold, should be less than one second
     * @param lockThreshold         How close the skystone needs to be to the center of the camera in order for us to stop (0.1 - 0.3)
     */
    //This loop uses the rear sensors to line up with a SkyStone
    public void SkystoneAlign(double moveSpeed, double wallDistance, double correctionCoefficient, double lockTime, double lockThreshold, double startRotation) {
        ElapsedTime deltaTime = new ElapsedTime();



//
        StopMovement();
    }

    /**
     * @param moveSpeed          How fast we will be moving forward (lower speeds recommended)
     * @param maxCorrectionAngle How fast we will correct our heading, in degrees
     * @param wallStopDistance   At what distance from the back wall will we stop moving
     */
    //This drives at a skystone while correcting itself
    public void DriveAtSkystone(double moveSpeed, double maxCorrectionAngle, double wallStopDistance, double startRotation) {

        StopMovement();
    }

    //Freezes the robots movement but continues to seek its correct rotation
    public void StopMovement() {
        robot.setPowerDouble4(0, 0, 0, 0, 0);
    }

    //Sends the 'message' to telemetry and updates it, mostly for C#-ness
    void print(String message) {
        telemetry.addData("", message);
        telemetry.update();
    }

    public void TransitionToTeleop() {
        //TODO FInISh HIM
    }

}
