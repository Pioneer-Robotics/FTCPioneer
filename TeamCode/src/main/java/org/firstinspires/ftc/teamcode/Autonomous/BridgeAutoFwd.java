package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "Skystone But Were Bad", group = "ftcPio")
public class BridgeAutoFwd extends Auto {

    @Override
    public void runOpMode() {
        startRobot();

        waitForStart();

        robot.driveByDistance(0, 1, 10, 3);

        StopMovement();
        StopRobot();
    }
}
