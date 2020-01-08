package org.firstinspires.ftc.teamcode;


import android.renderscript.Double2;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Auto;
import org.firstinspires.ftc.teamcode.Helpers.bMath;
import org.firstinspires.ftc.teamcode.Robot.Robot;
import org.firstinspires.ftc.teamcode.Robot.RobotArm;
import org.firstinspires.ftc.teamcode.Robot.RobotWallTrack;

@Autonomous(name = "fakeAuto", group = "Sensor")
public class FakeAutoBackup extends Auto {


    @Override
    public void runOpMode() {
        StartRobot();
        telemetry.addData("No Victory Fuckers!", "");
        telemetry.update();
        waitForStart();

        telemetry.addData("Smol Victory Fuckers!", "");
        telemetry.update();

        sleep(150);

        telemetry.addData("Minor Victory Fuckers!", "");
        telemetry.update();

        robot.DriveByDistancePoorly(1, 99.6);

        telemetry.addData("No Victory Mother Fuckers!", "");
        telemetry.update();

        double targetRotation = 90;

        robot.RotatePID(77.5, 1, 1000);
/*
        for (int i = 0; i < 100; i++) {
            robot.MoveComplex(new Double2(0, 0), 1, robot.GetRotation() - 90);

            if (Math.abs(robot.GetRotation() - targetRotation) < 2.5) {
                break;
            }
        }

 */
        telemetry.addData("Victory Mother Fuckers!", "");
        telemetry.update();

        robot.foundationServo0.setPosition(1);
        robot.foundationServo1.setPosition(0);

        StopMovement();
        StopRobot();
    }
}