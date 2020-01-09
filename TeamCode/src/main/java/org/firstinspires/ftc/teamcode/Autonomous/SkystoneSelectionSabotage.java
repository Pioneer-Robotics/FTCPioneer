package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Robot.RobotWallTrack;

@Autonomous(name = "Skystone Low Key Sabotage", group = "ftcPio")
public class SkystoneSelectionSabotage extends Auto {

    public double startRotation;

    ElapsedTime deltaTime = new ElapsedTime();

    double moveTime;

    boolean useLasers;

    @Override
    public void runOpMode() {
        StartRobot();

        waitForStart();

        GrabArm(0.5, 0.2);

//
//        if (useLasers) {
//            double targetDistance = 30;
//            double wallDistance = robot.wallTrack.sensorIDGroupPairs.get(RobotWallTrack.groupID.Group180).getDistanceAverage(DistanceUnit.CM);
//
//            while (Math.abs(wallDistance - targetDistance) >= 3) {
//                wallDistance = robot.wallTrack.sensorIDGroupPairs.get(RobotWallTrack.groupID.Group180).getDistanceAverage(DistanceUnit.CM);
//                robot.MoveComplex(Math.sin(wallDistance -), 0.5, robot.GetRotation() - robot.wallTrack.sensorIDGroupPairs.get(RobotWallTrack.groupID.Group180).getWallAngle());
//
//            }
//
//        } else {
//            robot.DriveByDistance(1, 30);
//        }
//
//        robot.RotatePID(-90, 1, 1000);
//
//        if (useLasers) {
//            double targetDistance = 30;
//            double wallDistance = robot.wallTrack.sensorIDGroupPairs.get(RobotWallTrack.groupID.Group180).getDistanceAverage(DistanceUnit.CM);
//
//            while (Math.abs(wallDistance - targetDistance) >= 3) {
//                wallDistance = robot.wallTrack.sensorIDGroupPairs.get(RobotWallTrack.groupID.Group180).getDistanceAverage(DistanceUnit.CM);
//                robot.MoveComplex(, 0.5, robot.GetRotation() - robot.wallTrack.sensorIDGroupPairs.get(RobotWallTrack.groupID.Group180).getWallAngle());
//
//            }
//        }
//        robot.DriveByDistance();


        DepositeArm(0.5, 0.2);


        StopMovement();
        StopRobot();
    }
}
