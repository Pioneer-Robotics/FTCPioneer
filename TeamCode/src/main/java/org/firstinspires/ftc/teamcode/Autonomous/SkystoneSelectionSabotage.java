package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Helpers.bTelemetry;

@Autonomous(name = "Skystone Low Key Sabotage", group = "ftcPio")
public class SkystoneSelectionSabotage extends Auto {

    private  double startRotation;

    private  ElapsedTime deltaTime = new ElapsedTime();

    private double moveTime;

    private final boolean lasers = true;

    @Override
    public void runOpMode() {
        StartRobot();

        startRotation = robot.getRotation();

        waitForStart();

//        robot.arm.SetGripState(RobotArm.GripState.CLOSED, 0.5);
//
//        sleep(10000);
//
//        robot.arm.SetGripState(RobotArm.GripState.IDLE, 0.5);
//        sleep(10000);
//        robot.arm.SetGripState(RobotArm.GripState.OPEN, 0.5);
//        sleep(10000);

        /*
        bTelemetry.print("Status: ", "Driving");
        robot.driveByDistancePoorly(0.5, Robot.simpleDirection.FORWARD, 1);

        bTelemetry.print("Status: ", "Grabbing");
        GrabArm(0.55, 0.35);
        bTelemetry.print("Status: ", "Rotating");

        if (side == FieldSide.SIDE_BLUE) {
            //-90
        } else {
            //90
        }
        */

        robot.rotatePIDRelative(-90, speed_high, 350);

        bTelemetry.print("Status: ", "Depositing");
        DepositeArm(0.55, 1);


        bTelemetry.print("Status: ", "Fixing Angle");

        robot.rotatePID(0, speed_high, 350);


// loop code for multiple skystones
//        for (int i = 0; i <= 2; i++) {
//            bTelemetry.print("Loop: ", Integer.toString(i));
//            bTelemetry.print("Status: ", "Grabbing");
//            GrabArm(0.5, 0.2);
//            bTelemetry.print("Status: ", "Rotating");
//            robot.rotatePIDRelative(-90, 0.8, 10000000);
//            bTelemetry.print("Status: ", "Fixing angle");
//            if (lasers) {
//                double distance = robot.wallTrack.sensorIDGroupPairs.get(RobotWallTrack.groupID.Group180).getDistanceAverage(DistanceUnit.CM);
//                while (Math.abs(distance - 90) >= 3 && !isStopRequested()) {
//                    robot.moveComplex(Math.copySign(90, -(distance - (90-i*20))), 0.4, 0, 0);
//                    distance = robot.wallTrack.sensorIDGroupPairs.get(RobotWallTrack.groupID.Group180).getDistanceAverage(DistanceUnit.CM);
//                }
//            }
//            robot.setPowerDouble4(0, 0, 0, 0, 0);
//            bTelemetry.print("Status: ", "Depositing");
//            DepositeArm(0.5, 0.2);
//
//
//            bTelemetry.print("Status: ", "Fixing Angle");
//            if (lasers) {
//                double angle = robot.wallTrack.sensorIDGroupPairs.get(RobotWallTrack.groupID.Group180).getWallAngle();
//                while (Math.abs(angle) >= 1 && !isStopRequested()) {
//                    robot.rotateSimple(Math.copySign(0.2, angle));
//                    angle = robot.wallTrack.sensorIDGroupPairs.get(RobotWallTrack.groupID.Group180).getWallAngle();
//                }
//            }
//            robot.setPowerDouble4(0, 0, 0, 0, 0);
//
//            bTelemetry.print("Status: ", "Fixing Distance");
//            if (lasers) {
//                double distance = robot.wallTrack.sensorIDGroupPairs.get(RobotWallTrack.groupID.Group180).getDistanceAverage(DistanceUnit.CM);
//                bTelemetry.print("Distance: ", Double.toString(distance));
//                while (Math.abs(distance - 30) >= 3 && !isStopRequested()) {
//                    robot.moveComplex(Math.copySign(90, -(distance - 30)), 0.4, 0, 0);
//                    distance = robot.wallTrack.sensorIDGroupPairs.get(RobotWallTrack.groupID.Group180).getDistanceAverage(DistanceUnit.CM);
//                }
//            }
//            robot.setPowerDouble4(0, 0, 0, 0, 0);
//            bTelemetry.print("Status: ", "Rotating");
//            robot.rotatePIDRelative(90, 0.8, 100000000);
//            bTelemetry.print("Status: ", "Fixing angle");
//            if (lasers) {
//                double angle = robot.wallTrack.sensorIDGroupPairs.get(RobotWallTrack.groupID.Group180).getWallAngle();
//                while (Math.abs(angle) >= 1 && !isStopRequested()) {
//                    robot.rotateSimple(Math.copySign(0.2, angle));
//                    angle = robot.wallTrack.sensorIDGroupPairs.get(RobotWallTrack.groupID.Group180).getWallAngle();
//                }
//                robot.setPowerDouble4(0, 0, 0, 0, 0);
//            }
//            bTelemetry.print("Status: ", "Fixing Distance");
//            if (lasers) {
//                double distance = robot.wallTrack.sensorIDGroupPairs.get(RobotWallTrack.groupID.Group180).getDistanceAverage(DistanceUnit.CM);
//                while (Math.abs(distance - 90) >= 3 && !isStopRequested()) {
//                    robot.moveComplex(Math.copySign(90, -(distance - (90-((i+1)*20)))), 0.4, 0, 0);
//                    distance = robot.wallTrack.sensorIDGroupPairs.get(RobotWallTrack.groupID.Group180).getDistanceAverage(DistanceUnit.CM);
//                }
//            }
//            robot.setPowerDouble4(0, 0, 0, 0, 0);
//
//
//        }
        StopMovement();
        StopRobot();
    }
}
