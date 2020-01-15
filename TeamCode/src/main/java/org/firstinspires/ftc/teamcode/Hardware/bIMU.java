package org.firstinspires.ftc.teamcode.Hardware;

import android.graphics.Path;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.Robot.RobotConfiguration;

import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicInteger;


//Used in place of BNO055IMU, it takes the average of both IMU's for readings
public class bIMU extends Thread {

    private BNO055IMU imu_0;
    private BNO055IMU imu_1;

    private BNO055IMU.Parameters IParameters = new BNO055IMU.Parameters();

//    OpMode op;

    public AtomicInteger initStatus = new AtomicInteger(0);

    public IMUStart imuStart0;
    public IMUStart imuStart1;

    public void Start(OpMode opMode) {
        initStatus.set(0);
        imu_0 = opMode.hardwareMap.get(BNO055IMU.class, RobotConfiguration.imu_0);
        imu_1 = opMode.hardwareMap.get(BNO055IMU.class, RobotConfiguration.imu_1);

        imuStart0 = new IMUStart(opMode, imu_0, false);
        imuStart1 = new IMUStart(opMode, imu_1, false);

        imuStart0.run();
        imuStart1.run();
    }

    public class IMUStart implements Runnable {

        public OpMode op;

        public BNO055IMU imu;

        boolean useLogging;

        public IMUStart(OpMode _op, BNO055IMU _imu, boolean useLogging) {
            op = _op;
            imu = _imu;
        }

        public void run() {
            IParameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
            IParameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
            IParameters.calibrationDataFile = "BNO055IMUCalibration.json";
            IParameters.loggingEnabled = useLogging;
            if (useLogging) {
                IParameters.loggingTag = "IMU" + Math.round(Math.random() * 1000);
            }

            imu.initialize(IParameters);
            initStatus.set(initStatus.get() + 1);
        }

    }

    //Returns the average of both IMU rotations
    public double getRotation(AngleUnit angleUnit) {
//        op.telemetry.addData("IMU 0: ", imu_0.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, angleUnit).firstAngle);
//        op.telemetry.addData("IMU 1: ", imu_1.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, angleUnit).firstAngle);

        double imuRotation_0 = imu_0.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, angleUnit).firstAngle;
        double imuRotation_1 = imu_1.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, angleUnit).firstAngle;

        if (imuRotation_0 - imuRotation_1 > 180) {
            imuRotation_1 += 360;
        } else if (imuRotation_1 - imuRotation_0 > 180) {
            imuRotation_0 += 360;
        }

//        op.telemetry.addData("IMU AVG: ", ((imuRotation_0 + imuRotation_1) / 2) % 360);


        return ((imuRotation_0 + imuRotation_1) / 2) % 360;
    }


}
