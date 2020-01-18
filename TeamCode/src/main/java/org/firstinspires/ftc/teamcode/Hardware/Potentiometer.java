package org.firstinspires.ftc.teamcode.Hardware;

import android.graphics.Path;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
@Deprecated
public class Potentiometer {

    public AnalogInput analogInput;

    //DISABLED
    public Potentiometer(OpMode opMode, String inputName) {
//        analogInput = opMode.hardwareMap.get(AnalogInput.class, inputName);
    }

    public Potentiometer() {
        analogInput = null;
    }

    public double getAngle() {
//        return -analogInput.getVoltage() / 0.0122222222;
        return 1;
    }
}
