package org.firstinspires.ftc.teamcode.Autonomous;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Skystone Foundation", group = "ftcPio")
public class SkystoneFoundation extends Auto {


    @Override
    public void runOpMode() {
        StartRobot();

        waitForStart();

        StopMovement();
        StopRobot();
    }
}