package org.firstinspires.ftc.teamcode.Autonomous;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Skystone Foundation Joe Blue", group = "ftcPio")
public class SkystoneFoundationJoeBlu extends Auto {

    private boolean endOnWall = true;

    @Override
    public void runOpMode() {
        StartRobot();
        waitForStart();
        //go forward 70 cm
        robot.driveByDistance(0, 0.5, 90);
        //go left 75 cm
        robot.driveByDistance(-80, 0.5, 70);
        //go forward 20 cm to correct
        robot.driveByDistance(0, 0.5, 20);
        //grip foundation
        robot.setFoundationGripperState(1);
        //delay
        sleep(2000);
        //pull it backwards
        robot.driveByDistance(180, 0.5, 120);
        //release servos
        robot.setFoundationGripperState(0);
        //go under bridge
        robot.driveByDistance(100, 1, 250);
        //pull into the wall, to make sure you're there. Or don't (if/else statement)
        if (endOnWall) {
            robot.driveByDistance(180, 1, 5);
        } else {
            robot.driveByDistance(0, 1, 50);
        }
        StopMovement();
        StopRobot();
    }
}