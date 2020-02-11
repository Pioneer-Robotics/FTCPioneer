package org.firstinspires.ftc.teamcode.TeleOp.ArmControls;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Helpers.bMath;
import org.firstinspires.ftc.teamcode.Robot.Robot;
import org.firstinspires.ftc.teamcode.Robot.RobotArm;

public class TeleopServosControls {

    public static double moveServosAndGetGripAngle(Robot robot,
                                            double lunchboxRot,
                                            double gripAngle,
                                            boolean idle,
                                            boolean grab,
                                            boolean gripFoundation) {
        robot.capstoneServo.setPosition(lunchboxRot);

        robot.foundationServo0.setPosition(gripFoundation ? 0.05 : 1);
        robot.foundationServo1.setPosition(gripFoundation ? 0.95 : 0);

        double _gripAngle = bMath.Clamp(gripAngle, 0, 180);

        if (idle) {
            robot.arm.setGripState(RobotArm.GripState.IDLE, _gripAngle / 180);
        } else if (grab) {
            robot.arm.setGripState(RobotArm.GripState.CLOSED, _gripAngle / 180);
        } else {
            robot.arm.setGripState(RobotArm.GripState.OPEN, _gripAngle / 180);
        }
        return _gripAngle;
    }

    /*
    hold A button to make the gripper point down
     */
    public static double pointGripperDown(Gamepad gamepad, Robot robot, double gripAngle) {
        double result = gripAngle;
        if (gamepad.a) {
            result = 90 - robot.arm.thetaAngle() - 10;
        }
        return result;
    }


    /*
    rotate gripper down with the left dpad
    */
    public static double rotateGripperDown(Gamepad gamepad, double gripAngle, ElapsedTime deltaTime) {
        double result = gripAngle;
        if (gamepad.left_bumper) {
            result += deltaTime.seconds() * 135 * 1.5;
        }
        return result;
    }

    /*
    rotate gripper up with the right dpad
    */
    public static double rotateGripperUp(Gamepad gamepad, double gripAngle, ElapsedTime deltaTime) {
        double result = gripAngle;
        if (gamepad.right_bumper) {
            result -= deltaTime.seconds() * 135 * 1.5;
        }
        return result;
    }

}
