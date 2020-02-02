package org.firstinspires.ftc.teamcode.Hardware;

import android.graphics.Path;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;

public class Potentiometer {
    private double regSlope = 133;
    private double regIntercept = -4.62;

    public AnalogInput analogInput;

    //DISABLED
    public Potentiometer(OpMode opMode, String inputName) {
        analogInput = opMode.hardwareMap.get(AnalogInput.class, inputName);
    }


    public double getAngle() {
        return analogInput.getVoltage()*regSlope - regIntercept;

    }
    public double getVoltage() {
        return analogInput.getVoltage();

    }

    public void addData(double angle) {

    }

    public void clearData() {

    }

}
