package org.firstinspires.ftc.teamcode.TeleOp.ArmControls;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Helpers.bMath;
import org.firstinspires.ftc.teamcode.Robot.Robot;
import org.firstinspires.ftc.teamcode.Robot.RobotConfiguration;
import org.firstinspires.ftc.teamcode.TeleOp.DriverControls.EngineeringControlData;

// **** Functions/Methods should attempt to only have 1 job. ****

// **** Attempt to pass in values, change things internally and then return a complete object. ****
//      Do NOT change the passed in object mid function since it's possible that it's being changed externally.
//      This allows for 1 action to happen on 1 object which eventually is easier to test and comprehend.

public class TeleopArmControls {

    // Public API

    // 1 job only. With this in mind This function's 1 job is to direct traffic. It doesn't know what's going on in the traffic just how to direct it.
    // This methods only directs traffic (ask for data from a different methods, determines whether the arm should be in a rectangular control state & asks a different method to calculate EngineeringControlData to return )

    public static EngineeringControlData updateArm(Gamepad gamePad,
                                                   Robot robot,
                                                   EngineeringControlData engiData,
                                                   ElapsedTime deltaTime,
                                                   Telemetry telemetry) {
        EngineeringControlData result = engiData;

        result = updateRectControls(gamePad, robot, result);

        //Allows the Gripper to be moved straight up and down with the right joystick
        if (result.rectControls) {
            result = updateArmForRecControls(gamePad, robot, result, telemetry);
        } else {
            result = updateArmForNonRecControls(gamePad, engiData, deltaTime, telemetry);
        }

        return result;
    }

    // Private Methods

    // 1 job only. With this in mind Rect controls and non-rec controls are in separate functions
    private static EngineeringControlData updateArmForRecControls(Gamepad gamePad,
                                                                  Robot robot,
                                                                  EngineeringControlData engiData,
                                                                  Telemetry telemetry) {
        telemetry.addLine("Arm Control: Rect");

        EngineeringControlData result = engiData;

        //Switch mode to position based extension
        result.powerExtension = false; // QUESTION - Should this be set to false in both Rec & non Rec ???

        //set power and distance to the Arm.
        result.extension = robot.arm.RectExtension(result.rectControls_goingUp, result.xExtConst, result.yExtConst);
        result.extension = bMath.Clamp(robot.arm.cmToTicks(result.extension) / RobotConfiguration.arm_ticksMax);
        result.raiseSpeed = result.rectControls_goingUp ? -0.5 * gamePad.right_stick_y : -0.5 * gamePad.right_stick_x;

        return result;
    }

    private static EngineeringControlData updateArmForNonRecControls(Gamepad gamePad,
                                                                     EngineeringControlData engiData,
                                                                     ElapsedTime deltaTime,
                                                                     Telemetry telemetry) {
        telemetry.addLine("Arm Control: Radial");

        EngineeringControlData result = engiData;

        //Switch mode to position based extension
        result.powerExtension = false;

        //
        result.extension += gamePad.right_trigger * deltaTime.seconds() * 1.5;    //extend arm when right trigger held and dpad left is pressed
        result.extension -= gamePad.left_trigger * deltaTime.seconds() * 1.5;     //retract arm when left trigger held and dpad left is pressed

        // This is the previous code without {} - I left it here for reference. I believe this one isn't a bug, but the one below in updateRectControls was
//        if (!result.spoolProtect) result.extension = bMath.Clamp(result.extension, 0, 1);
//        result.raiseSpeed = bMath.Clamp(-gamePad.left_stick_y, -1, 1); //set raise

        // See NOTE below - I'm assuming that raiseSpeed should always be set
        if (!result.spoolProtect) {
            result.extension = bMath.Clamp(result.extension, 0, 1);
        }
        result.raiseSpeed = bMath.Clamp(-gamePad.left_stick_y, -1, 1); //set raise


        return result;
    }

    /*
    This method calculates all rectangular control information and determines whether the arm should
    be in a rectangular control state.
     */

    // 1 job only. With this in mind, this method only calculates all rectangular control information
    private static EngineeringControlData updateRectControls(Gamepad gamePad,
                                                             Robot robot,
                                                             EngineeringControlData _engiData) {
        EngineeringControlData result = _engiData;

        // Activates rectControls when right stick is being moved
        result.rectControls = ((Math.abs(gamePad.right_stick_y) > 0.1) || (Math.abs(gamePad.right_stick_x) > 0.1));

        //sets direction of rectControls to whichever axis is greater
        result.rectControls_goingUp = Math.abs(gamePad.right_stick_y) > Math.abs(gamePad.right_stick_x);

        //get new extension constants if rectControls changes or if direction changes

        // ******** NOTE - generally not using {} after an if statement is frowned upon because it's a huge source of bugs (I added here)
        //          I also assumed that both `xExtConst` & `yExtConst` should be set if the if statement is true.
        //          As the computer read it before, it was `xExtConst` if the if statement was tru, but ALWAYS setting the `yExtConst`
        //          If you don't include {} the compiler only includes the immediate next line in the if statement.
        //          I'm almost positive this is true for Java. We can check it, or just use the {} to be certain.
        //          Many more modern languages than Java require that coders use {} after if statement just for this specific bug.
        if ((result.rectControls != result.rectControlsCheck) || (result.rectControls_goingUp != result.rectControls_goingUpCheck)) {
            result.xExtConst = robot.arm.xExtConst();
            result.yExtConst = robot.arm.yExtConst();
        }

        result.rectControlsCheck = result.rectControls;
        result.rectControls_goingUpCheck = result.rectControls_goingUp;

        return result;
    }
}
