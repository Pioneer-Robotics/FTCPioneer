package org.firstinspires.ftc.teamcode.Helpers;


import android.renderscript.Double4;

public class DrivePower {

    private double frontLeft;
    private double frontRight;
    private double backLeft;
    private double backRight;

    public DrivePower(double frontLeftPower, double frontRightPower, double backLeftPower, double backRightPower) {
        frontLeft = frontLeftPower;
        frontRight = frontRightPower;
        backLeft = backLeftPower;
        backRight = backRightPower;
    }

    public DrivePower() {
        frontLeft = 0;
        frontRight = 0;
        backLeft = 0;
        backRight = 0;
    }

    public void assign(double frontLeftPower, double frontRightPower, double backLeftPower, double backRightPower) {
        frontLeft = frontLeftPower;
        frontRight = frontRightPower;
        backLeft = backLeftPower;
        backRight = backRightPower;
    }

    public void multiply(double scalar) {
        frontLeft *= scalar;
        frontRight *= scalar;
        backLeft *= scalar;
        backRight *= scalar;
    }

    public void from(Double4 double4) {
        frontLeft = double4.x;
        frontRight = double4.y;
        backLeft = double4.z;
        backRight = double4.w;
    }

    @Deprecated
    public DrivePower normilize() {
        return null;
    }
}


