package org.firstinspires.ftc.teamcode.Troubleshooting.TestingOps;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Autonomous.VuforiaSkystoneDetector;
import org.firstinspires.ftc.teamcode.Helpers.PID;
import org.firstinspires.ftc.teamcode.Robot.Robot;

@Autonomous(name = "TestingOpMode1", group = "Trouble Shooting")
public class TestingOpMode1 extends LinearOpMode {

    public VuforiaSkystoneDetector t = new VuforiaSkystoneDetector();

    @Override
    public void runOpMode() {

        t.Start(this);

        waitForStart();


        while (opModeIsActive()) {
            t.Update(this);
        }

    }

}
