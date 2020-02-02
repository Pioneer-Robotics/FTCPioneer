package org.firstinspires.ftc.teamcode.CalibrationOps;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Autonomous.Auto;

@Autonomous(name = "Arm Pot Calibration", group = "Calibration")
public class ArmPotCalibration extends Auto {
    @Override
    public void runOpMode() {
        int datapoints = 10;
        startRobot();
        waitForStart();

        for (int i=0; i<datapoints; i++){
            robot.arm.setArmStateWait( ((double)i) / ((double) datapoints),0);
            robot.armPotentiometer.addData(robot.arm.derivedPotentiometerAngle(robot.arm.currentArmQuadBaseDistance()));

        }

        StopMovement();
        StopRobot();
    }
}