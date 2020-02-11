package org.firstinspires.ftc.teamcode.TeleOp.ArmControls;

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

}
