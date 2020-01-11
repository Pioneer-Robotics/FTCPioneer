package org.firstinspires.ftc.teamcode.Autonomous;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Robot.Robot;

@Autonomous(name = "fakeAuto", group = "Sensor")
public class FakeAutoBackup extends Auto {

    @Override
    public void runOpMode() {
        StartRobot();
        robot.SetFoundationGripperState(0);
        //go forward 70 cm
        robot.DriveByDistancePoorly(60, Robot.simpleDirection.FORWARD, 1);
        //go left 35 cm
        robot.DriveByDistancePoorly(35, Robot.simpleDirection.LEFT, 1);
        //lower servos
        robot.SetFoundationGripperState(1);
        int a = 0;
        while (a < 40000){
            a++;
        }
        //go left 6 more cm
        robot.DriveByDistancePoorly(6, Robot.simpleDirection.LEFT,0.5);
        //pull it backwards
        robot.DriveByDistancePoorly(50, Robot.simpleDirection.BACKWARD,1);
        //release foundation
        telemetry.addLine("should move servo");
        robot.SetFoundationGripperState(1);
        telemetry.addLine("servo moved?");
        telemetry.update();
        //go toward the bridge
        robot.DriveByDistancePoorly(50, Robot.simpleDirection.RIGHT,1);


        /*
        StartRobot();
        telemetry.addData("No Victory Fuckers!", "");
        telemetry.update();
        waitForStart();

        telemetry.addData("Small Victory Fuckers!", "");
        telemetry.update();

        sleep(150);

        telemetry.addData("Minor Victory Fuckers!", "");
        telemetry.update();

        robot.DriveByDistancePoorly(1, 99.6);

        telemetry.addData("No Victory Mother Fuckers!", "");
        telemetry.update();

        robot.RotatePID(77.5, 0.25, 1000);

        telemetry.addData("Victory Mother Fuckers!", "");
        telemetry.update();

        robot.foundationServo0.setPosition(0.8);
        robot.foundationServo1.setPosition(0.2);

        StopMovement();
        StopRobot();
         */
    }
}