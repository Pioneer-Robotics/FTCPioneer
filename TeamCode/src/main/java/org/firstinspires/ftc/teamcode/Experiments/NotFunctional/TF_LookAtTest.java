package org.firstinspires.ftc.teamcode.Experiments.NotFunctional;


import android.renderscript.Double4;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.Robot.Robot;
import org.firstinspires.ftc.teamcode.Helpers.bMath;

//10.6.19, proof of concept to move the robot to look at a skystone
@Autonomous(name = "LookAtDatSkyStone", group = "Sensor")
@Disabled
public class TF_LookAtTest extends LinearOpMode {
    Robot hwInf = new Robot();

    TF_ThreadTest TF_thread = new TF_ThreadTest();

    @Override
    public void runOpMode() throws InterruptedException {
//        telemetry.addData(":", "STARTING");

        hwInf.init(hardwareMap, this, true);
//        TF_thread.startFromOpmode(this);
//        TF_thread.start();

        hwInf.setDriveMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hwInf.setDriveMode(DcMotor.RunMode.RUN_USING_ENCODER);

        double rotationDelta = 0;

        //Loopy loop loop that loops
        while (opModeIsActive()) {
//            hwInf.setPowerDouble4(new Double4(-1, -1, 1, 1), 1);

            hwInf.moveComplex(90, 1, 0, 0);
//            telemetry.addData("Velocity ", hwInf.imu.getVelocity());
            telemetry.update();
            Recognition skystone = TF_thread.skyStone();
            if (skystone != null) {
                telemetry.addData("Found skystone!", "TRUE");
                float factor = skystone.getRight() - (skystone.getImageWidth() / 2);
                telemetry.addData("Float factor found : ", factor);
                rotationDelta = bMath.MoveTowards(rotationDelta, factor / skystone.getImageWidth(), 1);
                telemetry.addData("Rotation factor found : ", rotationDelta);

                hwInf.setPowerDouble4(new Double4(rotationDelta, -rotationDelta, rotationDelta, -rotationDelta), 1);


//                hwInf.moveComplex(0, 0, hwInf.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle + rotationDelta);
                telemetry.update();
            }

        }

        //shutdown the thread or die!
        TF_thread.stopThread();

        //shutdown the motors when the robots done doin what its doin.
        hwInf.setDriveMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

}
