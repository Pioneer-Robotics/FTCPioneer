package org.firstinspires.ftc.teamcode.Robot;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Helpers.PID;
import org.firstinspires.ftc.teamcode.Helpers.bMath;

import java.util.HashMap;
import java.util.HashSet;


//Called by the Robot.java to track along a flat surface. This surface is identified by inputting an angle value which correlates to the sensor group that will be used for tracking (look at the tape on the robot for anglez)
//Make sure to Trial and Error the hell out of this one!
//Might be fun to put this in Robot 'run'
public class RobotWallTrack {

    //List of all of our laser groups, mainly for ease of access
    public HashMap<groupID, SensorGroup> sensorIDGroupPairs = new HashMap<groupID, SensorGroup>();
    public Robot robot;


    //<editor-fold desc="Runtime data">
    AvoidanceConfiguration avoidanceConfig = new AvoidanceConfiguration();

    public SensorGroup currentGroup = new SensorGroup();

    double curDriveAngle = 0;
    double wallAngle = 0;


    double physicalOffset = 0;
    double driveAngle = 0;
    double correctionAngle = 0;
    double correctedDriveAngle = 0;
    //</editor-fold>

    public static class SensorGroup {

        public double distance;

        //The last real distance used, this is used if the sensors are not giving valid readings
        double lastDistance;

        //Once we have more than 3 failed readings (returning the last known value) return the true value (value.Infinity)
        int failedReadings;

        //The angle at which the sensors are located on the robot
        public double sensorAngle;

        public DistanceSensor[] distanceSensors = new DistanceSensor[2];

        public double cache_avgDistance;
        public double cache_wallAngle;

        public double cache_distanceA;
        public double cache_distanceB;


        public enum TripletType {
            Right,
            Left
        }

        //<editor-fold desc="Init setups">
        //dL = the left most sensor
        //dR = the right most sensor
        public SensorGroup(DistanceSensor dL, DistanceSensor dR, Double dist, double angle) {
            distanceSensors[0] = dL;
            distanceSensors[1] = dR;
            distance = dist;
            sensorAngle = angle;
        }

        public SensorGroup(OpMode opMode, String dL, String dR, Double dist, double angle) {
            distanceSensors[0] = opMode.hardwareMap.get(DistanceSensor.class, dL);
            distanceSensors[1] = opMode.hardwareMap.get(DistanceSensor.class, dR);
            distance = dist;
            sensorAngle = angle;
        }

        public SensorGroup() {
        }
        //</editor-fold>

        //<editor-fold desc="External return groups">
        //Returns the last valid input with a buffer of 3 inputs, this is because of the unreliable target surface (transparent plastic)
        public double getDistance(SensorGroup.TripletType type, DistanceUnit unit) {
//            if (isValid(type) || failedReadings >= 3) {
//                lastDistance = ;
//                failedReadings = 0;
//            } else {
//                failedReadings++;
//            }
            return sensor(type).getDistance(unit);
        }

        public double getDistanceAverage(DistanceUnit unit) {
            return (getDistance(SensorGroup.TripletType.Right, unit) + getDistance(SensorGroup.TripletType.Left, unit)) / 2;
        }

        public double getDistanceAverage(double distanceA, double distanceB) {
            return (distanceA + distanceB) / 2;
        }

        //Return a distance sensor from type
        public DistanceSensor sensor(SensorGroup.TripletType type) {
            return (type == SensorGroup.TripletType.Right ? distanceSensors[1] : distanceSensors[0]);
        }

        //Returns true if our input is valid, meaning that we hit something and have accurate data.
        public boolean isValid(SensorGroup.TripletType type) {
            return sensor(type).getDistance(DistanceUnit.CM) < 500;
        }

        //Gets all of the data and stores in cache_
        public void CacheAll(DistanceUnit unit) {
            cache_distanceA = getDistance(TripletType.Left, unit);
            cache_distanceB = getDistance(TripletType.Right, unit);
            cache_avgDistance = getDistanceAverage(cache_distanceA, cache_distanceB);
            cache_wallAngle = getWallAngle(cache_distanceA, cache_distanceB);
        }


        //The angle of the triangle formed by the diffidence in distance sensors
        //   A     B
        //   |     |
        //   |     |
        //   |     |
        //   | dist|
        //   |-----|
        //   |     /
        //   |    /
        //   |   /
        //   |  /
        //   |o/
        //   |/
        //ASCII's hard, check the journal for better pictures
        public double getWallAngle() {
            return bMath.toDegrees(Math.atan((getDistance(SensorGroup.TripletType.Right, DistanceUnit.CM) - getDistance(SensorGroup.TripletType.Left, DistanceUnit.CM)) / distance));
        }

        public double getWallAngle(double distanceA, double distanceB) {
            return bMath.toDegrees(Math.atan(distanceA - distanceB) / distance);
        }

        public double getWallAngleRelative() {
            return getWallAngle() - sensorAngle;
        }

//        //Returns true if the three sensors have hit a perfect (with in 5%, see "error") line, this can be used to check if there's another robot or obstacle near us
//        //Get bounds to work on the sensors
//        //Error is between 0 (0%) and 1 (100%)
//        public boolean isValid(double error) {
//            return true;
//        }


        //</editor-fold>
    }

    public static class AvoidanceConfiguration {

        public double targetAngle;

        public double currentDistance;

        //Target distance to track too
        public double targetDistance;

        //The amount of degrees to move in order to correct movement, 45 = fast, 0 = no correction, 25 recommended
        public double correctionScale;

        //Init the config with range bounds and scale
        public AvoidanceConfiguration(double _targetDistance, double _correctionScale, double _targetAngle) {
            targetDistance = _targetDistance;
            correctionScale = _correctionScale;
            targetAngle = _targetAngle;
        }


        public AvoidanceConfiguration() {
        }

        //Returns a number between -1 and 1 based on which way we need to move deh bot and how fast we need too
        public Double CorrectionCoefficient() {

            double factor = 0;

            factor = ((Math.abs(currentDistance)) / targetDistance) - 1;

            return factor;
        }

        //The angle that we wanna move in (additive)
        public Double targetDirection() {
            return (CorrectionCoefficient() * correctionScale);
        }

        //Called to update the current distance var
        public void SetCurrentDistance(double value) {
            currentDistance = value;
        }

    }

    public enum groupID {
        Group90,
        Group180,
        Group270,
        Group0
    }

    public HashSet<Rev2mDistanceSensor> sensors = new HashSet<Rev2mDistanceSensor>();


    public void Start(OpMode op) {

        robot = Robot.instance;

        sensorIDGroupPairs.put(groupID.Group90, new SensorGroup(op, RobotConfiguration.distanceSensor_90A, RobotConfiguration.distanceSensor_90B, RobotConfiguration.distance_90AB, 90));
        sensorIDGroupPairs.put(groupID.Group180, new SensorGroup(op, RobotConfiguration.distanceSensor_180A, RobotConfiguration.distanceSensor_180B, RobotConfiguration.distance_180AB, 180));
        sensorIDGroupPairs.put(groupID.Group270, new SensorGroup(op, RobotConfiguration.distanceSensor_270A, RobotConfiguration.distanceSensor_270B, RobotConfiguration.distance_270AB, -90));


        //Add all of the sensors to the map for easy access later
        sensors.add(op.hardwareMap.get(Rev2mDistanceSensor.class, RobotConfiguration.distanceSensor_90A));
        sensors.add(op.hardwareMap.get(Rev2mDistanceSensor.class, RobotConfiguration.distanceSensor_90B));
        sensors.add(op.hardwareMap.get(Rev2mDistanceSensor.class, RobotConfiguration.distanceSensor_180A));
        sensors.add(op.hardwareMap.get(Rev2mDistanceSensor.class, RobotConfiguration.distanceSensor_180B));
        sensors.add(op.hardwareMap.get(Rev2mDistanceSensor.class, RobotConfiguration.distanceSensor_270A));
        sensors.add(op.hardwareMap.get(Rev2mDistanceSensor.class, RobotConfiguration.distanceSensor_270B));

        //Set up the front sensor here so we can access it through groupID's
//        sensorIDGroupPairs.put(groupID.Group0, new SensorGroup(op, RobotConfiguration.distanceSensor_0A, RobotConfiguration.distanceSensor_0A, 1.0, 0));

    }

    //Moves relative to a wall with out any rotation lock
    //                                                         |
    //                                                         |
    //                                                         |
    //                       180                               |
    //                                                         |
    //                       |                                 |
    //                       |                                 |
    //                       |                                 |
    //                       |                                 |
    //    -90 _____________________________________ 90         |
    //                                                         |
    //                       0                                 |
    //an angle offset of 0 will move us along the walls normal, of 90 the robot moves to the right, of -90 left.
    //speed == how fast we move along the wall, also in which direction
    //distance == how far away from the wall should we be
    //bounds == +-distance how far are we allowed to be before correction (5cm seems reasonable)
    //correctionScale == think of it as how fast we correct our self (its an angle measure: 0 = no correction, 90 == max correction), leave it around 25.
    @Deprecated

    public void MoveAlongWallSimple(groupID group, double speed, double distance, double bounds, double correctionScale, double angleOffset) {
        MoveAlongWallSimple(sensorIDGroupPairs.get(group), speed, distance, bounds, correctionScale, angleOffset);
    }

    @Deprecated
    public void MoveAlongWallSimple(SensorGroup group, double speed, double distance, double bounds, double correctionScale, double angleOffset) {

        //Configure the avoidance config
        avoidanceConfig = new AvoidanceConfiguration(distance, correctionScale, angleOffset);

        //Set up the group
        currentGroup = group;

        //Get the current sensors wall angle
        wallAngle = currentGroup.getWallAngle();

        //send our current world distance to the avoidance config
        avoidanceConfig.SetCurrentDistance(currentGroup.getDistanceAverage(DistanceUnit.CM));

        //Add the avoidance offset to our wall angle (to maintain the 'distance' from the wall)
        curDriveAngle = angleOffset - wallAngle + avoidanceConfig.CorrectionCoefficient();

        //MOVE
        robot.MoveSimple(angleOffset - wallAngle - avoidanceConfig.targetDirection(), speed);
    }


    /**
     * @param group                  What group of sensor the bot will use for wall tracking, these are labeled based on their angle to the robot
     * @param speed                  How fast are robot going to be moving along the wall (effects all movement)
     * @param distance               How far away the robot is going to try to stay away from the wall
     * @param correctionFactor       How fast the bot is going to correct our mistakes (distance) (1 == slow correction, not sensitive enough. 10 == speedy correction, needs testing but probs okay to use)
     * @param maxCorrectionMagnitude The max amount the bot can correct by (in degrees).
     * @param angleOffset            The angle the bot will be moving along the wall. If you need help finding this number ask Ben
     * @param rotationAngle          The angle that the bot will try to stay at.
     */
    public void MoveAlongWallComplex(groupID group, double speed, double distance, double correctionFactor, double maxCorrectionMagnitude, double angleOffset, double rotationAngle) {


        //get the physical angle these sensors are at to offset from movement
        physicalOffset = group == groupID.Group90 ? 90 : (group == groupID.Group180 ? 180 : (group == groupID.Group270 ? -90 : 0));

        //Set up the group
        currentGroup = sensorIDGroupPairs.get(group);

        //Get the current sensors wall angle
        wallAngle = currentGroup.getWallAngle();

        //Configure the avoidance config
        avoidanceConfig = new AvoidanceConfiguration(distance, maxCorrectionMagnitude, angleOffset - wallAngle + physicalOffset);

        //send our current world distance to the avoidance config
        avoidanceConfig.SetCurrentDistance(currentGroup.getDistanceAverage(DistanceUnit.CM));


        Robot.instance.Op.telemetry.addData("Move offset ", angleOffset);
        Robot.instance.Op.telemetry.addData("Real offset ", physicalOffset);

        //Add the avoidance offset to our wall angle (to maintain the 'distance' from the wall)
        double driveAngle = angleOffset - wallAngle + physicalOffset;

        //correctionAngle > 0 ? (avoidanceConfig.CorrectionCoefficient() > 0 ? Math.toRadians(physicalOffset - 180) : Math.toRadians(physicalOffset)) : (avoidanceConfig.CorrectionCoefficient() > 0 ? Math.toRadians(physicalOffset) : Math.toRadians(physicalOffset - 180))
        double correctionAngle = 0;
        if (angleOffset > 0) {
            if (avoidanceConfig.CorrectionCoefficient() > 0) {
                correctionAngle = Math.toRadians(physicalOffset);
            } else {
                correctionAngle = Math.toRadians(physicalOffset - 180);
            }
        } else {
            if (avoidanceConfig.CorrectionCoefficient() > 0) {
                correctionAngle = Math.toRadians(physicalOffset);
            } else {
                correctionAngle = Math.toRadians(physicalOffset - 180);
            }
        }
        double correctedDriveAngle = Math.toDegrees(bMath.MoveTowardsRadian(Math.toRadians(driveAngle), correctionAngle, Math.toRadians(bMath.Clamp(90 * Math.abs(avoidanceConfig.CorrectionCoefficient() * correctionFactor), 0, maxCorrectionMagnitude))));

        Robot.instance.Op.telemetry.addData("Drive Pre Correction", driveAngle);
        Robot.instance.Op.telemetry.addData("Correction CoNumb", avoidanceConfig.CorrectionCoefficient());
        Robot.instance.Op.telemetry.addData("Correction Number", correctionAngle);
        Robot.instance.Op.telemetry.addData("Corrected to ", correctedDriveAngle);
        Robot.instance.Op.telemetry.addData("Corrected by ", driveAngle - correctedDriveAngle);
        Robot.instance.Op.telemetry.addData("Current Distnace ", robot.GetDistance(groupID.Group180, DistanceUnit.CM));
        Robot.instance.Op.telemetry.addData("Current Angle ", wallAngle);
        Robot.instance.Op.telemetry.addData("Current Distnace Goal ", distance);

        //Move while keeping our rotation angle the same
        robot.MoveComplex(correctedDriveAngle, speed, robot.GetRotation() - rotationAngle);
    }

    ElapsedTime debuggingDeltaTime = new ElapsedTime();

    public void MoveAlongWallComplexPID(groupID group, double speed, double distance, PID controller, double maxCorrectionMagnitude, double angleOffset, double rotationAngle) {

        debuggingDeltaTime.reset();
//Store the 2 distances used this cycle to avoid 300ms delays on wall tracking

        //get the physical angle these sensors are at to offset from movement
        physicalOffset = group == groupID.Group90 ? 90 : (group == groupID.Group180 ? 180 : (group == groupID.Group270 ? -90 : 0));

        Robot.instance.Op.telemetry.addData("Set physical offset", debuggingDeltaTime.seconds());
        debuggingDeltaTime.reset();


        //Set up the group
        currentGroup = sensorIDGroupPairs.get(group);
        currentGroup.CacheAll(DistanceUnit.CM);


        Robot.instance.Op.telemetry.addData("Set current group", debuggingDeltaTime.seconds());
        debuggingDeltaTime.reset();

        //Get the current sensors wall angle
        wallAngle = currentGroup.cache_wallAngle;


        Robot.instance.Op.telemetry.addData("Got current angle", debuggingDeltaTime.seconds());
        debuggingDeltaTime.reset();

        //Configure the avoidance config
//        avoidanceConfig = new AvoidanceConfiguration(distance, maxCorrectionMagnitude, angleOffset - wallAngle + physicalOffset);

        avoidanceConfig.targetDistance = distance;
        avoidanceConfig.correctionScale = maxCorrectionMagnitude;
        avoidanceConfig.targetAngle = angleOffset - wallAngle + physicalOffset;

        Robot.instance.Op.telemetry.addData("Assigned config", debuggingDeltaTime.seconds());
        debuggingDeltaTime.reset();


        //send our current world distance to the avoidance config
        avoidanceConfig.SetCurrentDistance(currentGroup.cache_avgDistance);


        Robot.instance.Op.telemetry.addData("Updated config", debuggingDeltaTime.seconds());

        debuggingDeltaTime.reset();


//        Robot.instance.Op.telemetry.addData("Move offset ", angleOffset);
//        Robot.instance.Op.telemetry.addData("Real offset ", physicalOffset);

        //Add the avoidance offset to our wall angle (to maintain the 'distance' from the wall)
        driveAngle = angleOffset - wallAngle + physicalOffset;

        //correctionAngle > 0 ? (avoidanceConfig.CorrectionCoefficient() > 0 ? Math.toRadians(physicalOffset - 180) : Math.toRadians(physicalOffset)) : (avoidanceConfig.CorrectionCoefficient() > 0 ? Math.toRadians(physicalOffset) : Math.toRadians(physicalOffset - 180))
        correctionAngle = 0;
        if (angleOffset > 0) {
            if (avoidanceConfig.CorrectionCoefficient() < 0) {
                correctionAngle = bMath.toRadians(physicalOffset);
            } else {
                correctionAngle = bMath.toRadians(physicalOffset - 180);
            }
        } else {
            if (avoidanceConfig.CorrectionCoefficient() < 0) {
                correctionAngle = bMath.toRadians(physicalOffset);
            } else {
                correctionAngle = bMath.toRadians(physicalOffset - 180);
            }
        }
        correctedDriveAngle = Math.toDegrees(bMath.MoveTowardsRadian(bMath.toRadians(driveAngle), correctionAngle, bMath.toRadians(bMath.Clamp(Math.abs(controller.Loop(distance, currentGroup.cache_avgDistance)), 0, maxCorrectionMagnitude))));
        Robot.instance.Op.telemetry.addData("Angle correction", debuggingDeltaTime.seconds());
        Robot.instance.Op.telemetry.addData("Angle ", debuggingDeltaTime.seconds());

//        Robot.instance.Op.telemetry.addData("Current Distnace ", currentGroup.getDistanceAverage(DistanceUnit.CM));
//        Robot.instance.Op.telemetry.addData("Current Error", distance - currentGroup.getDistanceAverage(DistanceUnit.CM));
//        Robot.instance.Op.telemetry.addData("PID value", controller.State());

        //Move while keeping our rotation angle the same
        // 0 - 0 - 180 + 0
        // 0 - 0 - 90 + 0
        // 90 - 0 + 180 + 0
        // 0 - 0 + 90 + 0
        // -90 - 0 + 180
        // 90 - 0 + 180
        //0 - 0 - 90
        //-90 - 0 + 90
        //-90 -
        robot.MoveComplex(correctedDriveAngle, speed, robot.GetRotation() - rotationAngle);

    }

    public void MoveAlongWallComplex(groupID group, double speed, double angleOffset, double rotationAngle) {


        //get the physical angle these sensors are at to offset from movement
        physicalOffset = group == groupID.Group90 ? 90 : (group == groupID.Group180 ? 180 : (group == groupID.Group270 ? -90 : 0));

        //Set up the group
        currentGroup = sensorIDGroupPairs.get(group);

        //Get the current sensors wall angle
        wallAngle = currentGroup.getWallAngle();

        //Move while keeping our rotation angle the same
        robot.MoveComplex(angleOffset - wallAngle + physicalOffset, speed, robot.GetRotation() - rotationAngle);
    }


    //Returns the sensor group that has the smallest average reading
    public SensorGroup closestGroup() {


        SensorGroup group = null;
        double lowestDistance = 1000000;

        //Itterate threw all of our sensor pairs and find the one with the smallest current distance
        for (SensorGroup currentGroup : sensorIDGroupPairs.values()) {
            if (lowestDistance < currentGroup.getDistanceAverage(DistanceUnit.CM)) {
                lowestDistance = currentGroup.getDistanceAverage(DistanceUnit.CM);
                group = currentGroup;
            }
        }

        //return group with smallest reading
        return group;
    }


}
