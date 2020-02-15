package org.firstinspires.ftc.teamcode.CalibrationOps;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Autonomous.Auto;
import org.firstinspires.ftc.teamcode.Helpers.bMath;


@Autonomous(name = "Arm Pot Calibration", group = "Calibration")
public class ArmPotCalibration extends Auto {


    private ElapsedTime timer = new ElapsedTime();

    @Override
    public void runOpMode() {



        startRobot();
        waitForStart();




        for (int i = 0; i < robot.armPotentiometer.datapoints; i++) {

            robot.arm.setArmStateWait(((double) i) / ((double) robot.armPotentiometer.datapoints), 0);



            timer.reset();
            while (timer.seconds() < 0.5) { }
            robot.armPotentiometer.addData(robot.arm.derivedPotentiometerAngle(robot.arm.currentArmQuadBaseDistance()));

            timer.reset();
            while (timer.seconds() < 0.5) { }

            if (i>1){
                robot.armPotentiometer.calcRegression();
            }

            telemetry.addData("x[i]:",robot.armPotentiometer.voltages[i]);
            telemetry.addData("y[i]:",robot.armPotentiometer.angles[i]);
            telemetry.addData("arrayPos",robot.armPotentiometer.arrayPos);
            telemetry.addData("D value (mm):",robot.arm.currentArmQuadBaseDistance());
            telemetry.addData("Derived angle:", bMath.toDegrees(robot.arm.derivedPotentiometerAngle(robot.arm.currentArmQuadBaseDistance())));
            telemetry.addData("Voltage:",robot.armPotentiometer.getVoltage());
            telemetry.addData("Measured Angle:", bMath.toDegrees(robot.armPotentiometer.getAngle()));
            telemetry.addData("Regression Slope:",bMath.toDegrees(robot.armPotentiometer.regSlope));
            telemetry.addData("Regression Intercept:",bMath.toDegrees(robot.armPotentiometer.regIntercept));
            telemetry.addData("x2Sum", robot.armPotentiometer.x2Sum);
            telemetry.addData("xSum",robot.armPotentiometer.xSum);
            telemetry.addData("ySum", robot.armPotentiometer.ySum);
            telemetry.addData("xySum",robot.armPotentiometer.xySum);
            telemetry.update();

        }
        robot.armPotentiometer.calcRegression();
        robot.armPotentiometer.saveCalibrationData(robot.dataManger);

        robot.arm.setArmStateWait(0, 0);





        StopMovement();
        StopRobot();
    }
}
