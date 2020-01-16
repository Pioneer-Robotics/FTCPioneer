package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Robot.RobotArm;
import org.firstinspires.ftc.teamcode.Robot.RobotWallTrack;
@Disabled
@Deprecated
@Autonomous(name = "Skystone", group = "ftcPio")
public class SkystoneAuto extends Auto {

    public double startRotation;

    @Override
    public void runOpMode() {
        StartRobot();

        startRotation = robot.getRotation();


        waitForStart();

        robot.arm.setArmStateWait(0.1, 0, 1);

        //If we can't see the skystone, move forward a tad to get a better reading
        while (opModeIsActive()) {
//            if (jobs.tensorFlowaJob.getCurrentRecognition() == null) {
//                robot.driveByDistance(speed_low, 2.5);
//                sleep(500);
//            }
//            if (jobs.tensorFlowaJob.getCurrentRecognition() != null) {
//                StopMovement();
//                break;
//            }
        }


        //Line up with a skystone
        //A lockThreshold of .25 will get is within 19.5 degrees of the stone
        SkystoneAlign(speed_low, 50, 2, 0.5, 0.075, startRotation);

        //Drive forward while adjusting heading to line up with the skystone
        DriveAtSkystone(speed_med, 35, 15, startRotation);

//        jobs.tensorFlowaJob.shutdown();

        //Extend the arm 35% of the way
        robot.arm.SetArmState(0.1, 0.35, 1);
        sleep(100);

        //Deploy the claw and open it all the way
        robot.arm.SetGripState(RobotArm.GripState.OPEN, 0.5);
        sleep(250);

        //Drop the arm, hopefully, on a sky stone
        robot.arm.SetArmState(0, 0.35, 1);
        sleep(250);

        //Close the gripper!
        robot.arm.SetGripState(RobotArm.GripState.CLOSED, 0.5);
        sleep(250);

        //Raise the arm again to avoid dragging the stone on the ground
        robot.arm.SetArmState(0, 0.15, 1);

        //Roll back a wee bit
        robot.driveByDistance(speed_med, -5);

        //Rotate based on our side to face the back sensors towards the foundation
        if (side == FieldSide.SIDE_BLUE) {
            robot.rotatePID(90, speed_med, 100000);
        } else {
            robot.rotatePID(-90, speed_med, 10000);
        }


        //Move towards the foundation by wall tracking along the wall
        ResetWallPID();
        while (opModeIsActive() && robot.getDistance(RobotWallTrack.groupID.Group180, DistanceUnit.CM) > 100) {
            if (side == FieldSide.SIDE_BLUE) {
                robot.wallTrack.MoveAlongWallComplexPID(RobotWallTrack.groupID.Group90, speed_med, 36, walltrackingController, 35, -90, robot.getRotation());
            } else {
                robot.wallTrack.MoveAlongWallComplexPID(RobotWallTrack.groupID.Group270, speed_med, 36, walltrackingController, 35, 90, robot.getRotation());

            }

            if (!opModeIsActive()) {
                break;
            }
        }

        StopMovement();

        //Rotate to face the foundation
        robot.rotatePID(0, speed_high, 10000);

        //Lower the arm to latch to the foundation
        robot.arm.SetArmState(0, 0.35, 1);

        //Rotate based on our side to face the back sensors towards the foundation
        if (side == FieldSide.SIDE_BLUE) {
            robot.rotatePID(startRotation + 90, speed_med, 100000);
        } else {
            robot.rotatePID(startRotation - 90, speed_med, 10000);
        }
        StopRobot();
    }
}
