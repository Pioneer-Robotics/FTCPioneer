package org.firstinspires.ftc.teamcode.TeleOp.DriverControls;

public class DriverTeleopData {
    public double moveSpeed;
    public RotationData rotationData;
    public boolean movementModeToggleCheck;
    public boolean coordinateSystemLock;

    public DriverTeleopData(double moveSpeed,
                            RotationData rotationData,
                            boolean movementModeToggleCheck,
                            boolean coordinateSystemLock) {
        this.moveSpeed = moveSpeed;
        this.rotationData = rotationData;
        this.movementModeToggleCheck = movementModeToggleCheck;
        this.coordinateSystemLock = coordinateSystemLock;
    }
}