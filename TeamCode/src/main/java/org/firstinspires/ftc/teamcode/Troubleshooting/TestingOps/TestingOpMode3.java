package org.firstinspires.ftc.teamcode.Troubleshooting.TestingOps;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Robot.Robot;
import org.firstinspires.ftc.teamcode.Robot.RobotWallTrack;

@Autonomous(name = "TestingOpMode3", group = "Debugging")
//Used to ensure that all wheels are set up correctly
public class TestingOpMode3 extends LinearOpMode {

    private Robot robot = new Robot();


    private  ElapsedTime deltaTime = new ElapsedTime();


    private double targetRotation;

    @Override
    public void runOpMode() {
        robot.init(this, true);

        waitForStart();
        targetRotation = robot.getRotation();
        while (opModeIsActive()) {
            robot.wallTrack.MoveAlongWallComplex(RobotWallTrack.groupID.Group180, 0.2, 20, 3, 90, 90, targetRotation);
            telemetry.update();
        }

        robot.shutdown();
    }

}
