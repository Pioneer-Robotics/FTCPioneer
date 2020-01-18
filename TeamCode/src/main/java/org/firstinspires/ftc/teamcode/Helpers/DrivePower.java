package org.firstinspires.ftc.teamcode.Helpers;



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

    public void multiply(double scale) {
        frontLeft *= scale;
        frontRight *= scale;
        backLeft *= scale;
        backRight *= scale;
    }
}


