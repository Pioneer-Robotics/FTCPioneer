package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Helpers.bTelemetry;
import org.firstinspires.ftc.teamcode.Robot.RobotWallTrack;

@Autonomous(name = "Skystone Low Key Sabotage", group = "ftcPio")
public class SkystoneSelectionSabotage extends Auto {

    public double startRotation;

    ElapsedTime deltaTime = new ElapsedTime();

    double moveTime;

    final boolean lasers = true;

    @Override
    public void runOpMode() {
        StartRobot();

        startRotation = robot.GetRotation();

        waitForStart();

        bTelemetry.Print("Status: ", "Driving");
        robot.DriveByDistancePoorly(0.5, 20);

        //loop code for multiple skystones
        for (int i = 0; i <= 2; i++) {
            bTelemetry.Print("Loop: ", Integer.toString(i));
            bTelemetry.Print("Status: ", "Grabbing");
            GrabArm(0.5, 0.2);
            bTelemetry.Print("Status: ", "Rotating");
            robot.RotatePIDRelative(-90, 0.8, 10000000);
            bTelemetry.Print("Status: ", "Fixing angle");
            if (lasers) {
                double distance = robot.wallTrack.sensorIDGroupPairs.get(RobotWallTrack.groupID.Group180).getDistanceAverage(DistanceUnit.CM);
                while (Math.abs(distance - 90) >= 3) {
                    robot.MoveComplex(Math.copySign(90, -(distance - (90-i*20))), 0.4, 0);
                    distance = robot.wallTrack.sensorIDGroupPairs.get(RobotWallTrack.groupID.Group180).getDistanceAverage(DistanceUnit.CM);
                }
            }
            robot.SetPowerDouble4(0, 0, 0, 0, 0);
            bTelemetry.Print("Status: ", "Depositing");
            DepositeArm(0.5, 0.2);


            bTelemetry.Print("Status: ", "Fixing Angle");
            if (lasers) {
                double angle = robot.wallTrack.sensorIDGroupPairs.get(RobotWallTrack.groupID.Group180).getWallAngle();
                while (Math.abs(angle) >= 1) {
                    robot.RotateSimple(Math.copySign(0.2, angle));
                    angle = robot.wallTrack.sensorIDGroupPairs.get(RobotWallTrack.groupID.Group180).getWallAngle();
                }
            }
            robot.SetPowerDouble4(0, 0, 0, 0, 0);

            bTelemetry.Print("Status: ", "Fixing Distance");
            if (lasers) {
                double distance = robot.wallTrack.sensorIDGroupPairs.get(RobotWallTrack.groupID.Group180).getDistanceAverage(DistanceUnit.CM);
                bTelemetry.Print("Distance: ", Double.toString(distance));
                while (Math.abs(distance - 30) >= 3) {
                    robot.MoveComplex(Math.copySign(90, -(distance - 30)), 0.4, 0);
                    distance = robot.wallTrack.sensorIDGroupPairs.get(RobotWallTrack.groupID.Group180).getDistanceAverage(DistanceUnit.CM);
                }
            }
            robot.SetPowerDouble4(0, 0, 0, 0, 0);
            bTelemetry.Print("Status: ", "Rotating");
            robot.RotatePIDRelative(90, 0.8, 100000000);
            bTelemetry.Print("Status: ", "Fixing angle");
            if (lasers) {
                double angle = robot.wallTrack.sensorIDGroupPairs.get(RobotWallTrack.groupID.Group180).getWallAngle();
                while (Math.abs(angle) >= 1) {
                    robot.RotateSimple(Math.copySign(0.2, angle));
                    angle = robot.wallTrack.sensorIDGroupPairs.get(RobotWallTrack.groupID.Group180).getWallAngle();
                }
                robot.SetPowerDouble4(0, 0, 0, 0, 0);
            }
            bTelemetry.Print("Status: ", "Fixing Distance");
            if (lasers) {
                double distance = robot.wallTrack.sensorIDGroupPairs.get(RobotWallTrack.groupID.Group180).getDistanceAverage(DistanceUnit.CM);
                while (Math.abs(distance - 90) >= 3) {
                    robot.MoveComplex(Math.copySign(90, -(distance - (90-((i+1)*20)))), 0.4, 0);
                    distance = robot.wallTrack.sensorIDGroupPairs.get(RobotWallTrack.groupID.Group180).getDistanceAverage(DistanceUnit.CM);
                }
            }
            robot.SetPowerDouble4(0, 0, 0, 0, 0);


        }
        StopMovement();
        StopRobot();
    }
}
