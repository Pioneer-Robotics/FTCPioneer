package org.firstinspires.ftc.teamcode.CalibrationOps;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Autonomous.Auto;


@Autonomous(name = "Arm Pot Calibration", group = "Calibration")
public class ArmPotCalibration extends Auto {

    private final int datapoints = 10;
    private ElapsedTime deltaTime = new ElapsedTime();

    @Override
    public void runOpMode() {

        startRobot();
        waitForStart();
        robot.armPotentiometer.clearData();
        for (int i = 0; i < datapoints; i++) {

            robot.arm.setArmStateWait(((double) i) / ((double) datapoints), 0);
            deltaTime.reset();
            while (deltaTime.seconds() < 1) {
            }
            robot.armPotentiometer.addData(robot.arm.derivedPotentiometerAngle(robot.arm.currentArmQuadBaseDistance()));

        }
        robot.armPotentiometer.calcRegression();
        robot.armPotentiometer.saveCalibrationData(robot.dataManger);
        StopMovement();
        StopRobot();
    }
}