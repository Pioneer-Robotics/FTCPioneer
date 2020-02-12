package org.firstinspires.ftc.teamcode.Hardware;

import android.graphics.Path;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Helpers.bDataManager;
import org.firstinspires.ftc.teamcode.Helpers.bMath;

import java.util.HashMap;
import java.util.Map;

public class Potentiometer {

    //Exposed for telemetry data
    public double regSlope = bMath.toRadians(133);
    public double regIntercept = bMath.toRadians(-4.62);

    public final int datapoints = 10;
    private double[] voltages = new double[datapoints];
    private double[] angles = new double [datapoints];
    private int arrayPos = 0;


    public AnalogInput analogInput;

    //DISABLED
    public Potentiometer(OpMode opMode, String inputName ) {
        analogInput = opMode.hardwareMap.get(AnalogInput.class, inputName);
    }


    public double getAngle() {
        return analogInput.getVoltage() * regSlope + regIntercept;
    }

    public double getVoltage() {
        return analogInput.getVoltage();
    }

    public void addData(double angle) {
        voltages[arrayPos] = getVoltage();
        angles[arrayPos] = angle;
        arrayPos++;
    }


    public void calcRegression() {
        double xSum=0; //angles
        double ySum=0; //voltage
        double x2Sum=0; //x squared sum
        double xySum=0;

        for (int i = 0; i < arrayPos; i++) {
            xSum += angles[i];
            ySum += voltages[i];
            x2Sum += angles[i] * angles[i];
            xySum += angles[i] * voltages[i];
        }

        regSlope = (arrayPos*xySum - xSum*ySum) / (arrayPos*x2Sum - xSum*xSum);
        regIntercept = (ySum*x2Sum - xSum*xySum) / (arrayPos*x2Sum - x2Sum);
    }

    public void saveCalibrationData(bDataManager dataManager) {
        dataManager.writeData("pot_reg_slope", regSlope);
        dataManager.writeData("pot_reg_intercept", regIntercept);
    }
}

