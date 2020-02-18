package org.firstinspires.ftc.teamcode.TeleOp.DriverControls;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Helpers.Vector2;
import org.firstinspires.ftc.teamcode.Helpers.bMath;
import org.firstinspires.ftc.teamcode.Robot.Robot;

public class TeleopDriverControls {

    // Private properties
    private static double fullRotation = 360;
    private static final double sq2 = bMath.sq2();

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
        DriverTeleopData result = new DriverTeleopData(updateMoveSpeed,
                updateRotationData,
                updateMovementModeToggleCheck,
                updateCoordinateSystemLock);

        return result;
    }

    /*
    These methods determine the power levels for the wheels
     */
    public static double getLeftDiagPower(Vector2 moveDirection,
                                           Robot robot,
                                           boolean useLockedRotation,
                                           double rotationLockAngle,
                                           Telemetry telemetry) {

        double movementInput_x = moveDirection.x;
        double movementInput_y = moveDirection.y;

        // drive code
        if (useLockedRotation) {
//            telemetry.addData("Drive System", "New");
//
//            Vector2 movementVectorCache_left = robot.getMovementVector(gamepad, rotationLockAngle, movementInput_x, movementInput_y);
//            return ((-movementVectorCache_left.y + movementVectorCache_left.x) / sq2);
            return 0;
        } else {
            telemetry.addData("Drive System", "Old");

            return ((-movementInput_y + movementInput_x) / sq2);
        }
    }

    public static double getRightDiagPower(Vector2 moveDirection,
                                            Robot robot,
                                            boolean useLockedRotation,
                                            double rotationLockAngle,
                                            Telemetry telemetry) {

        double movementInput_x = moveDirection.x;
        double movementInput_y = moveDirection.y;

        if (useLockedRotation) {
//            telemetry.addData("Drive System", "New");
//
//            Vector2 movementVectorCache_right = robot.getMovementVector(gamepad, rotationLockAngle, movementInput_x, movementInput_y);
//            return ((-movementVectorCache_right.y - movementVectorCache_right.x) / sq2);
            return 0;
        } else {
            telemetry.addData("Drive System", "Old");
            return ((-movementInput_y - movementInput_x) / sq2);
        }
    }

    // Private Methods
    // Spped Boost or Slow Motion
    private static double moveSpeedForBoostOrSlowMotion(Gamepad gamePad) {
        //let left bumper toggle boost vs slow mode on the right trigger for fine control of the robot
        if (!gamePad.left_bumper) {
            //trigger makes robot slower
            double m = 0.7;
            return m - (0.125 * m) * (gamePad.left_trigger + gamePad.right_trigger)
                    - (0.125 * m) * bMath.squared(gamePad.left_trigger + gamePad.right_trigger);

        } else {
            //trigger makes robot faster
            return bMath.Clamp(0.5 + gamePad.right_trigger / 2.0, 0, 1);
        }
    }

    // D pad
    private static RotationData updateDPadRotationData(Gamepad gamePad,
                                                       RotationData rotationData) {
        //left and right dpad can shift "front" of robot by 90 degrees in coordinate lock mode
        RotationData result = rotationData;
        result.rotationLockAngle = ((result.rotationLockAngle + 450) % fullRotation) - fullRotation;

        result = updateDPadLeft(gamePad, result);
        result = updateDPadRight(gamePad, result);
        return result;
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
