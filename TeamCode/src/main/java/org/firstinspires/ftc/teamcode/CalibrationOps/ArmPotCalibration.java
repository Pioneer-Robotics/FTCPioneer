package org.firstinspires.ftc.teamcode.CalibrationOps;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Autonomous.Auto;


@Autonomous(name = "Arm Pot Calibration", group = "Calibration")
public class ArmPotCalibration extends Auto {


    private ElapsedTime timer = new ElapsedTime();

    @Override
    public void runOpMode() {

        telemetry.addData("D value:",robot.arm.currentArmQuadBaseDistance());
        telemetry.addData("Derived angle:",robot.arm.derivedPotentiometerAngle(robot.arm.currentArmQuadBaseDistance()));
        telemetry.addData("Voltage:",robot.armPotentiometer.getVoltage());
        telemetry.addData("Measured Angle:", robot.armPotentiometer.getAngle());
        telemetry.addData("Regression Slope:",robot.armPotentiometer.regSlope);
        telemetry.addData("Regression Intercept:",robot.armPotentiometer.regIntercept);

        startRobot();
        waitForStart();

        for (int i = 0; i < robot.armPotentiometer.datapoints; i++) {

            robot.arm.setArmStateWait(((double) i) / ((double) robot.armPotentiometer.datapoints), 0);
            timer.reset();
            while (timer.seconds() < 0.5) { }
            robot.armPotentiometer.addData(robot.arm.derivedPotentiometerAngle(robot.arm.currentArmQuadBaseDistance()));
            timer.reset();
            while (timer.seconds() < 0.5) { }

            if (i>2){
                robot.armPotentiometer.calcRegression();
            }
        }
        robot.armPotentiometer.calcRegression();
        robot.armPotentiometer.saveCalibrationData(robot.dataManger);
        StopMovement();
        StopRobot();
    }
}