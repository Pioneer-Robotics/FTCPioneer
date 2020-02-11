package org.firstinspires.ftc.teamcode.TeleOp;


import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Hardware.bMotor;
import org.firstinspires.ftc.teamcode.Helpers.Vector2;
import org.firstinspires.ftc.teamcode.Helpers.bMath;
import org.firstinspires.ftc.teamcode.Robot.Robot;
import org.firstinspires.ftc.teamcode.Robot.RobotArm;
import org.firstinspires.ftc.teamcode.Robot.RobotConfiguration;
import org.firstinspires.ftc.teamcode.TeleOp.ArmControls.TeleopArmControls;
import org.firstinspires.ftc.teamcode.TeleOp.DriverControls.DriverTeleopData;
import org.firstinspires.ftc.teamcode.TeleOp.DriverControls.EngineeringControlData;
import org.firstinspires.ftc.teamcode.TeleOp.DriverControls.RotationData;
import org.firstinspires.ftc.teamcode.TeleOp.DriverControls.TeleopDriverControls;

/// Advice-
/// Consolidate State variable into a similar concept object. (see EngineeringControlData for example etc...)
/// Move helper functions into Classes that relate (see Robot updateRobotDrive() for example, getMovementVector() still needs to be moved to a Math class)
/// Create layers of abstration to the code that makes it easier to read and understand (see TeleopDriverControls for example)
/// In general, attempt to create sub-layers of abstractions that have code that will almost never change so that if can be added to the code base and then confidently ignored as working flawlessly. (Adding self testing code with a testing framework would also be very helpful)
/// Adding layers of abstrations allow the coder to only focus on the level of code that needs to be updated (or read and understood) without unnecessary added complexity.


@TeleOp(name = "TeleOp", group = "Sensor")
public class Teleop extends TeleOpMode {


    private Robot robot = new Robot();

    private ElapsedTime deltaTime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);


    //Program state

    //Driver Control Variables
    private double moveSpeed;

    private boolean leftRotateCoordCheck = false;
    private boolean rightRotateCoordCheck = false;

    private double leftDiagPower = 0;
    private double rightDiagPower = 0;
    private final double sq2 = Math.pow(2, 1 / 2);
    private double leftRotatePower = 0;
    private double rightRotatePower = 0;

    private double rotationLockAngle = 0;

    private boolean movementModeToggleCheck = false;
    private boolean coordinateSystemLock = false;

    //Arm Control Variables
//    private double raiseSpeed = 0;
//    private double extension = 0;
    private double gripAngle = 180;

    //Rectangular Control Variables New
    /// Code clean up advice --- State variable that have a similar concept should be grouped into a single object
    private EngineeringControlData engiData = new EngineeringControlData();

    //Gripper Control
    private boolean grab = true; //whether the gripper is not gripping
    private boolean bButton2Check = false; //prevState of grab

    private boolean idle = false; //whether the gripper is in rest position
    private boolean xButton2Check = false;

    private boolean dropLunchBox = false;
    private boolean yButton2Check = false;

    private double lunchboxRot = 0.5;

    private boolean gripFoundation = false;
    private boolean bButton1Check = false;
    //Mode Switch Variables

    // Additional properties
    double frontLeftWheelPower;
    double frontRightWheelPower;
    double backLeftWheelPower;
    double backRightWheelPower;

    Vector2 movementVectorCache_left = new Vector2(0, 0);
    Vector2 movementVectorCache_right = new Vector2(0, 0);

    RotationData rotationData = new RotationData(0, false, false);
    DriverTeleopData driverTeleopData = new DriverTeleopData(0, new RotationData(0, false, false), false, false);


    // Life Cycle Methods
    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(this, false);

        robot.arm.rotationMode = RobotArm.ArmThreadMode.Disabled;
        robot.arm.extensionMode = RobotArm.ArmThreadMode.Disabled;
        robot.arm.length.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        for (bMotor motor : robot.driveManager.driveMotors) {
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        gamepad1.setJoystickDeadzone(0.1f);


        waitForStart();

        lunchboxRot = 1;
        robot.arm.setGripState(RobotArm.GripState.IDLE, 0);
//        robot.arm.setGripState(RobotArm.GripState.IDLE, 60);
        gripAngle = 180;
        while (opModeIsActive()) {

            updateDriverControls();

            updateArm();

            updateServoControls();

            moveServos();

            doTelemetry();

            // Update deltaTime
            deltaTime.reset();
        }
        robot.shutdown();
    }

    /*
    This method updates and applies any changes to the driver controls and handles movement
     */
    private void updateDriverControls() {
        ///DRIVER CONTROLS

        // Create driverTeleopData to get values to update robot state
//        rotationData = new RotationData(rotationLockAngle, leftRotateCoordCheck, rightRotateCoordCheck);
        rotationData.rotationLockAngle = rotationLockAngle;
        rotationData.leftRotateCoordCheck = leftRotateCoordCheck;
        rotationData.rightRotateCoordCheck = rightRotateCoordCheck;
        driverTeleopData = TeleopDriverControls.setupDriverController(gamepad1,
                robot,
                rotationData,
                movementModeToggleCheck,
                coordinateSystemLock);

        // Handle returned data values here
        moveSpeed = driverTeleopData.moveSpeed;
        movementModeToggleCheck = driverTeleopData.movementModeToggleCheck;
        coordinateSystemLock = driverTeleopData.coordinateSystemLock;
        rotationLockAngle = driverTeleopData.rotationData.rotationLockAngle;
        leftRotateCoordCheck = driverTeleopData.rotationData.leftRotateCoordCheck;
        rightRotateCoordCheck = driverTeleopData.rotationData.rightRotateCoordCheck;

        // Update Diag Power
        leftDiagPower = getLeftDiagPower(coordinateSystemLock, gamepad1.left_stick_x, gamepad1.left_stick_y, 90, 90);
        rightDiagPower = getRightDiagPower(coordinateSystemLock, gamepad1.left_stick_x, gamepad1.left_stick_y, 90, 90);

        leftRotatePower = gamepad1.right_stick_x;
        rightRotatePower = -gamepad1.right_stick_x;

        // Update Robot Drive
        frontLeftWheelPower = moveSpeed * (leftDiagPower + leftRotatePower);
        frontRightWheelPower = moveSpeed * (rightDiagPower + rightRotatePower);
        backLeftWheelPower = moveSpeed * (rightDiagPower + leftRotatePower);
        backRightWheelPower = moveSpeed * (leftDiagPower + rightRotatePower);
        robot.updateRobotDrive(frontLeftWheelPower, frontRightWheelPower, backLeftWheelPower, backRightWheelPower);
    }

    // TODO: - Move to ??? Class
    /*
    This method determines the power levels for the wheels
     */
    private double getLeftDiagPower(boolean useLockedRotation,
                                    double movementInput_x,
                                    double movementInput_y,
                                    double currentRobotRotation,
                                    double targetRotation) {
        // drive code
        if (useLockedRotation) {
            telemetry.addData("Drive System", "New");

            movementVectorCache_left = robot.getMovementVector(gamepad1, rotationLockAngle, movementInput_x, movementInput_y);
            return ((-movementVectorCache_left.y + movementVectorCache_left.x) / sq2);
        } else {
            telemetry.addData("Drive System", "Old");

            return ((-gamepad1.left_stick_y + gamepad1.left_stick_x) / sq2);
        }

    }

    private double getRightDiagPower(boolean useLockedRotation,
                                     double movementInput_x,
                                     double movementInput_y,
                                     double currentRobotRotation,
                                     double targetRotation) {

        if (useLockedRotation) {
            telemetry.addData("Drive System", "New");

            movementVectorCache_right = robot.getMovementVector(gamepad1, rotationLockAngle, movementInput_x, movementInput_y);
            return ((-movementVectorCache_right.y - movementVectorCache_right.x) / sq2);

        } else {
            telemetry.addData("Drive System", "Old");
            return ((-gamepad1.left_stick_y - gamepad1.left_stick_x) / sq2);
        }
    }


    // All Moved to Robot.java
    // I left this commented out for reference for now (remove after read and understood)

    // ******* Also I may have found a BUG ****** Look in the method in Robot to fix it IF necessary **************

    /*
    // *** These variables are always set internally to the methods, so they should be scoped to the function
//    double _newGamepadX;
//    double _newGamepadY;

    // *** These variables never leave the internal methods either and are always set in the fuction be fore they are returned in Vector2
//    double newGamepadX;
//    double newGamepadY;

    // All of these variables also never leave the method and should be scoped only internally to the methods. (Not class wide)
//    double movementSpeed;
//    double angle;
//    Vector2 movementVectorCache = new Vector2(0, 0);

    // TODO: - Move to a Robot Class
    private Vector2 getMovementVector(double movementInput_x, double movementInput_y) {
        // A common pattern is to name the object that will be returned `result` so that it can be easily understood
        // With this in mind I'm renaming `movementVectorCache` to result since that is the Result object of this method

        Vector2 result = new Vector2(0, 0);

        double angle = Math.toRadians(robot.getRotation() - rotationLockAngle);

        double movementSpeed = (Math.sqrt(Math.pow(movementInput_x, 2) + Math.pow(movementInput_y, 2)));

        // *** These should be scoped to the method since they are only used in the method and are always reset before use in the methods
        //      In General try to scope variables to as small as scope as possible. This helps alleviate bugs

        double _newGamepadX = movementSpeed * Math.cos(angle + Math.atan(movementInput_y / movementInput_x));
        double _newGamepadY = movementSpeed * Math.sin(angle + Math.atan(movementInput_y / movementInput_x));

        // *** Variables `newGamepadX` & `newGamepadY` never leave the internal methods either and are always set in the fuction be fore they are returned in Vector2
        //      I also colapsed them directly into `result.x` & `result.y` since other then for logging purposes they were unnecessary.
        result.x = (gamepad1.left_stick_x <= 0) ? -_newGamepadX : _newGamepadX;
        result.y = (gamepad1.left_stick_x <= 0) ? -_newGamepadY : _newGamepadY; // **** IS `gamepad1.left_stick_x` a BUG **** shouldn't it be y ???
        return result;
    }
*/

    /*
    Asks TeleopArmControls for the updated arm data (engiData) and then calls move arm
    */
    private void updateArm() {
        EngineeringControlData result = TeleopArmControls.updateArm(gamepad2, robot, engiData, deltaTime, telemetry);
        moveArm(robot, result);
    }

    /*
    Moves the arm to position specified by the engiData
    */
    private void moveArm(Robot robot,
                         EngineeringControlData engiData) {
        if (engiData.powerExtension)
            robot.arm.SetArmStateExtensionPower(engiData.extendSpeed, engiData.raiseSpeed);
        else robot.arm.SetArmStatePower(engiData.extension, engiData.raiseSpeed);
    }

    /*

     */
    private void updateServoControls() {

        //press the X button to put the grabber in "idle" position
        if (gamepad2.x && !xButton2Check) {
            idle = true;
        }
        xButton2Check = gamepad2.x;

        //press the B button to open or close grabber
        if (gamepad2.b && !bButton2Check) {
            if (idle) {
                grab = false; //if it's in idle, pressing "B" should open it
            } else {
                grab = !grab;
            }
            idle = false;
        }
        bButton2Check = gamepad2.b;

        //hold A button to make the gripper point down
        if (gamepad2.a) {
            gripAngle = 90 - robot.arm.thetaAngle() - 10;
        }

        //rotate gripper down with the left dpad
        if (gamepad2.left_bumper) {
            gripAngle += deltaTime.seconds() * 135 * 1.5;
        }

        //rotate gripper up with the right dpad
        if (gamepad2.right_bumper) {
            gripAngle -= deltaTime.seconds() * 135 * 1.5;
        }

        //move foundation grippers with b button
        if (gamepad1.b && !bButton1Check) gripFoundation = !gripFoundation;
        bButton1Check = gamepad1.b;

        if (gamepad2.y && !yButton2Check) dropLunchBox = !dropLunchBox;
        yButton2Check = gamepad2.y;

        if (dropLunchBox) lunchboxRot = 0;
        else lunchboxRot = 0.738;

        if (gamepad2.dpad_down && !engiData.spoolProtectCheck)
            engiData.spoolProtect = !engiData.spoolProtect;
        robot.arm.setSpoolProtect(engiData.spoolProtect);
        engiData.spoolProtectCheck = gamepad2.dpad_down;

    }


    private void moveServos() {
        robot.capstoneServo.setPosition(lunchboxRot);

        robot.foundationServo0.setPosition(gripFoundation ? 0.05 : 1);
        robot.foundationServo1.setPosition(gripFoundation ? 0.95 : 0);

        gripAngle = bMath.Clamp(gripAngle, 0, 180);
        if (idle) {
            robot.arm.setGripState(RobotArm.GripState.IDLE, gripAngle / 180);
        } else if (grab) {
            robot.arm.setGripState(RobotArm.GripState.CLOSED, gripAngle / 180);
        } else {
            robot.arm.setGripState(RobotArm.GripState.OPEN, gripAngle / 180);
        }

    }

    private void doTelemetry() {
//        telemetry.addLine("------ Control ------");
//
//        telemetry.addData("robot arm extension state", robot.arm.extensionMode.toString());
//        telemetry.addData("robot arm extension state", robot.arm.rotationMode.toString());
//
        telemetry.addData("deltaTime", deltaTime.milliseconds());
//        telemetry.addLine("------ Movement ------");
        telemetry.addData("Rotation Locked ", coordinateSystemLock);
//        telemetry.addData("Current Rotation ", robot.getRotation());
////            telemetry.addData("Offset Angle ", angle);
//        telemetry.addLine("-------- Arm  --------");
//        telemetry.addData("Current Arm Angle", bMath.toDegrees(robot.arm.thetaAngle()));
//        telemetry.addData("Current Potentiometer angle", robot.armPotentiometer.getAngle());
//        telemetry.addData("Rotation Position", robot.arm.rotation.getCurrentPosition() / RobotConfiguration.arm_rotationMax);
        telemetry.addData("RectWanted?:", engiData.rectControls);
//        telemetry.addData("target spool position", engiData.extension * RobotConfiguration.arm_ticksMax);
//        telemetry.addData("spool position", robot.arm.length.getCurrentPosition());
//        telemetry.addData("spool position as percent", robot.arm.length.getCurrentPosition() / RobotConfiguration.arm_ticksMax);
//        telemetry.addData("arm rotation as percent", robot.arm.rotation.getCurrentPosition() / RobotConfiguration.arm_rotationMax);
        telemetry.addData("spoolProtect", engiData.spoolProtect);
//        telemetry.addLine("------ Lunchbox ------");
        telemetry.addData("Current Lunchbox", lunchboxRot);
        telemetry.update();
    }


}



