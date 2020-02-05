package org.firstinspires.ftc.teamcode.Autonomous;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "DevFoundationBlue", group = "ftcPio")
public class DevFoundationBlueSide extends Auto {
    @Override
    public void runOpMode() {
        boolean endOnWall = true;
        startRobot();
        waitForStart();
        sleep(2000);
        //ready gripper to grab foundation
        robot.gripFoundation();
        //go forward 70 cm
        robot.driveByDistance(0, 0.5, 90);
        //go left 75 cm
        robot.driveByDistance(-75, 0.5, 70);
        //go forward 25 cm to correct
        robot.driveByDistance(5, 0.5, 30);
        //grip foundation
        robot.releaseFoundation();
        //delay
        sleep(2000);
        //pull it backwards
        robot.driveByDistance(180, 0.5, 125);
        //
        robot.driveByDistance(165, 0.25, 5);
        //go a tad forward to push gripper off foundation
        robot.driveByDistance(0, 0.25, 5);
        //release servos
        robot.gripFoundation();
        //go under bridge
        robot.driveByDistance(100, 0.5, 225);
        //pull into the wall, to make sure you're there. Or don't (if/else statement)
        if (endOnWall) {
            //robot.rotatePID(32, 0.75,200);
            //sleep(5000);
            robot.driveByDistance(180, 0.5, 25);
        } else {
            robot.driveByDistance(0, 1, 50);
        }
        StopMovement();
        StopRobot();
    }
}