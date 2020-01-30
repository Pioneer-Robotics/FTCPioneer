package org.firstinspires.ftc.teamcode.Autonomous;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "DevFoundationRed", group = "ftcPio")
public class DevFoundationRedSide extends Auto {
    @Override
    public void runOpMode() {
        boolean endOnWall = true;
        boolean real = true; //DO NOT mess with
        startRobot();
        waitForStart();
        if (real) {
            //ready gripper to grab foundation
            robot.releaseFoundation();
            //go forward 70 cm
            robot.driveByDistance(0, 0.5, 90);
            //go right 75 cm
            robot.driveByDistance(80, 0.5, 70);
            //go forward 20 cm to correct
            robot.driveByDistance(0, 0.5, 20);
            //grip foundation
            robot.gripFoundation();
            //delay
            sleep(2000);
            //pull it backwards
            robot.driveByDistance(180, 0.5, 120);
            //go a tad forward to push gripper off foundation
            robot.driveByDistance(0, 0.25, 5);
            //release servos
            robot.releaseFoundation();
            //go under bridge
            robot.driveByDistance(-90, 1, 225);
            //pull into the wall, to make sure you're there. Or don't (if/else statement)
            if (endOnWall) {
                robot.driveByDistance(180, 1, 5);
            } else {
                robot.driveByDistance(0, 1, 50);
            }
        }

        //else is for testing specific things
        //doesn't always work
        else {
            robot.releaseFoundation();
            robot.driveByDistance(90, 0.5, 60);
            sleep(3000);
        }
        StopMovement();
        StopRobot();
    }
}