package org.firstinspires.ftc.teamcode.TeleOp.DriverControls;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.Helpers.bMath;
import org.firstinspires.ftc.teamcode.Robot.Robot;

public class TeleopDriverControls {

    // Private properties
    private static double fullRotation = 360;

    // Public API
    public static DriverTeleopData setupDriverController(Gamepad gamePad,
                                                         Robot robot,
                                                         RotationData rotationData,
                                                         boolean movementModeToggleCheck,
                                                         boolean coordinateSystemLock) {
        // Result Objects
        double updateMoveSpeed;
        RotationData updateRotationData = rotationData;
        boolean updateMovementModeToggleCheck = movementModeToggleCheck;
        boolean updateCoordinateSystemLock = coordinateSystemLock;

        // create update speed object
        updateMoveSpeed = moveSpeedForBoostOrSlowMotion(gamePad);

        // reset the "front" of the robot to be the real front
        if (gamePad.a) {
            updateRotationData.rotationLockAngle = robot.getRotation();
        }
        // up/down of right stick can incrementally change which way is the "front" of the robot in coordinate lock mode
        updateRotationData.rotationLockAngle = ((updateRotationData.rotationLockAngle + 3.0 * gamePad.right_stick_y + fullRotation) % fullRotation) - fullRotation;

        updateRotationData = updateDPadRotationData(gamePad, updateRotationData);

        // y button toggles coordinate system lock
        if (gamePad.y && !movementModeToggleCheck) {
            updateRotationData.rotationLockAngle = robot.getRotation();
            updateCoordinateSystemLock = !coordinateSystemLock;
        }
        updateMovementModeToggleCheck = gamePad.y;

        // Return 4 updated Objects
        DriverTeleopData driverTelopData = new DriverTeleopData(updateMoveSpeed,
                updateRotationData,
                updateMovementModeToggleCheck,
                updateCoordinateSystemLock);

        return driverTelopData;
    }

    // Private Methods
    // Spped Boost or Slow Motion
    private static double moveSpeedForBoostOrSlowMotion(Gamepad gamePad) {
        //let left bumper toggle boost vs slow mode on the right trigger for fine control of the robot
        if (!gamePad.left_bumper) {
            //trigger makes robot slower
            return bMath.Clamp(0.25 * (0.5 * (1 - gamePad.right_trigger) + (1 - gamePad.left_trigger) + 0.5), 0, 1);
        } else {
            //trigger makes robot faster
            return bMath.Clamp(0.5 + gamePad.right_trigger / 2, 0, 1);
        }
    }

    // D pad
    private static RotationData updateDPadRotationData(Gamepad gamePad,
                                                       RotationData rotationData) {
        //left and right dpad can shift "front" of robot by 90 degrees in coordinate lock mode
        RotationData updatedRotationData = rotationData;
        updatedRotationData.rotationLockAngle = ((updatedRotationData.rotationLockAngle + 450) % fullRotation) - fullRotation;

        updatedRotationData = updateDPadLeft(gamePad, updatedRotationData);
        updatedRotationData = updateDPadRight(gamePad, updatedRotationData);
        return updatedRotationData;
    }

    private static RotationData updateDPadLeft(Gamepad gamePad, RotationData rotationData) {
        //left dpad can shift "front" of robot by 90 degrees in coordinate lock mode
        RotationData result = rotationData;
        if (gamePad.dpad_left && !rotationData.leftRotateCoordCheck) {
            result.rotationLockAngle = rotationData.rotationLockAngle;
        }
        result.leftRotateCoordCheck = gamePad.dpad_left;
        return result;
    }

    private static RotationData updateDPadRight(Gamepad gamePad, RotationData rotationData) {
        //right dpad can shift "front" of robot by 90 degrees in coordinate lock mode
        RotationData result = rotationData;
        if (gamePad.dpad_right && !rotationData.rightRotateCoordCheck) {
            result.rotationLockAngle = rotationData.rotationLockAngle;
        }
        result.rightRotateCoordCheck = gamePad.dpad_right;
        return result;
    }
}
