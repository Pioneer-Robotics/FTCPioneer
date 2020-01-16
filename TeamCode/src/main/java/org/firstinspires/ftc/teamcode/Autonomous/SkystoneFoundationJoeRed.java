package org.firstinspires.ftc.teamcode.Autonomous;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Skystone Foundation Joe Red", group = "ftcPio")
public class SkystoneFoundationJoeRed extends Auto {

    private boolean endOnWall = true;

    @Override
    public void runOpMode() {
        StartRobot();
        robot.setFoundationGripperState(0);
        waitForStart();
        //go forward 70 cm
        robot.driveByDistance(0, 0.5, 90);
        //go right 75 cm, (check)
        robot.driveByDistance(80, 0.5, 60);
        //go forward 20 cm to correct
        robot.driveByDistance(0, 0.5, 20);
        //grip foundation
        robot.setFoundationGripperState(1);
        //delay
        sleep(2000);
        robot.driveByDistance(180, 0.5, 100);

        robot.rotatePID(-35, 1, 3);
        robot.setFoundationGripperState(0);
        sleep(2000);
        robot.rotatePID(0, 1, 3);

        robot.driveByDistance(180, 0.5, 50);
//        robot.driveByDistance(180, 0.5, 120);

        //pull it backwards
//        robot.driveByDistance(180, 0.5, 120);
        //release servos
//        robot.setFoundationGripperState(0);
        //go under bridge (check)
        robot.driveByDistance(-100, 1, 250);
        //pull into the wall, to make sure you're there. Or don't (if/else statement)
        if (endOnWall) {
            robot.driveByDistance(180, 1, 5);
        } else {
            robot.rotatePID(0, 1, 2);
            robot.driveByDistance(0, 1, 150);
        }
        StopMovement();
        StopRobot();
    }
}