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

    private Map regSet = new HashMap();

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
        regSet.put(angle, getVoltage());
    }

    public void clearData() {
        regSet.clear();
    }

    public void calcRegression() {
        regSlope = bMath.toRadians(133);
        regIntercept = bMath.toRadians(-4.62); //TODO add actual regression here
    }

    public void saveCalibrationData(bDataManager dataManager) {
        dataManager.writeData("pot_reg_slope", regSlope);
        dataManager.writeData("pot_reg_intercept", regIntercept);
    }
}
