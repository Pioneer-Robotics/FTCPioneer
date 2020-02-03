package org.firstinspires.ftc.teamcode.TeleOp.DriverControls;

public class RotationData {
    public double rotationLockAngle;
    public boolean leftRotateCoordCheck;
    public boolean rightRotateCoordCheck;

    public RotationData(double rotationLockAngle, boolean leftRotateCoordCheck, boolean rightRotateCoordCheck) {
        this.rotationLockAngle = rotationLockAngle;
        this.leftRotateCoordCheck = leftRotateCoordCheck;
        this.rightRotateCoordCheck = rightRotateCoordCheck;
    }
}