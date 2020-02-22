package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Hardware.bMotor;
import org.firstinspires.ftc.teamcode.Helpers.Vector2;
import org.firstinspires.ftc.teamcode.Helpers.bMath;
import org.firstinspires.ftc.teamcode.Robot.Robot;
import org.firstinspires.ftc.teamcode.Robot.RobotArm;
import org.firstinspires.ftc.teamcode.TeleOp.ArmControls.TeleopArmControls;
import org.firstinspires.ftc.teamcode.TeleOp.ArmControls.TeleopServosControls;
import org.firstinspires.ftc.teamcode.TeleOp.DriverControls.DriverTeleopData;
import org.firstinspires.ftc.teamcode.TeleOp.DriverControls.EngineeringControlData;
import org.firstinspires.ftc.teamcode.TeleOp.DriverControls.RotationData;
import org.firstinspires.ftc.teamcode.TeleOp.DriverControls.TeleopDriverControls;


@TeleOp(name = "TeleOp", group = "Sensor")
public class Teleop extends TeleOpMode {
    private boolean tankControlsVeryFast = false; //TODO integrate with the rest of how these bools tend to work
    private boolean xButton1Check = false; //TODO make like the above also

    /*
    this is the to help the tankControlls initialize properly
    what it is here doesn't really matter, it's set to false as soon as you hit "play"
     */
    private boolean cycledQuestionMark = false; //TODO reformat this to the new way we format variables
    private boolean snapAngle_functionWentHorriblyWrong = false; //this boolean exists purely for debugging
    //TODO delete the snapAngle_functionWentHorriblyWrong boolean before merging
    private Robot robot = new Robot();

    //Program State
    private ElapsedTime deltaTime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

    //Driver Control State Variables
    private boolean movementModeToggleCheck = false;
    private boolean coordinateSystemLock = false;
    private double rotationLockAngle = 0;
    private boolean leftRotateCoordCheck = false;
    private boolean rightRotateCoordCheck = false;

    private EngineeringControlData engiData = new EngineeringControlData();

    //Gripper Control
    private double gripAngle = 180;

    private boolean grab = true; //whether the gripper is not gripping
    private boolean bButton2Check = false; //prevState of grab

    private boolean idle = false; //whether the gripper is in rest position
    private boolean xButton2Check = false;

    private boolean dropLunchBox = false;
    private boolean yButton2Check = false;

    private double lunchboxRot = 0.5;

    private boolean gripFoundation = false;
    private boolean bButton1Check = false;
    private double movespeedout = 0;
    private double leftfrontpowerOut = 0;

    // ************** Life Cycle Methods **************
    @Override
    public void runOpMode() throws InterruptedException {
        preStartSetup();

        robot.updateRobotDrive(0, 0, 0, 0);

        waitForStart();

        robot.updateRobotDrive(0, 0, 0, 0);
        lunchboxRot = 1;
        robot.arm.setGripState(RobotArm.GripState.IDLE, 0);
//        robot.arm.setGripState(RobotArm.GripState.IDLE, 60);
        gripAngle = 180;

        while (opModeIsActive()) {
            updateDriverControls();
            updateArm();
            updateServoControls();
            doTelemetry(telemetry, deltaTime, coordinateSystemLock, engiData, lunchboxRot, snapAngle_functionWentHorriblyWrong);
            deltaTime.reset(); // Update deltaTime
            cycledQuestionMark = true; //this while loop has now ran at least once
        }

        robot.shutdown();
    }

    // Private Methods
    private void preStartSetup() {
        robot.init(this, false);

        robot.arm.rotationMode = RobotArm.ArmThreadMode.Disabled;
        robot.arm.extensionMode = RobotArm.ArmThreadMode.Disabled;
        robot.arm.length.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        for (bMotor motor : robot.driveManager.driveMotors) {
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        gamepad1.setJoystickDeadzone(0.1f);
    }

    // ************** Driver Methods **************

    /*
    This method updates and applies any changes to the driver controls and handles movement
     */
    private void updateDriverControls() {
        for (bMotor motor : robot.driveManager.driveMotors) {
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
        ///DRIVER CONTROLS

        // Create driverTeleopData to get values to update robot state
        RotationData rotationData = new RotationData(0, false, false);
        rotationData.rotationLockAngle = rotationLockAngle;
        rotationData.leftRotateCoordCheck = leftRotateCoordCheck;
        rotationData.rightRotateCoordCheck = rightRotateCoordCheck;

        DriverTeleopData driverTeleopData = new DriverTeleopData(0, new RotationData(0, false, false), false, false);
        driverTeleopData = TeleopDriverControls.setupDriverController(gamepad1,
                robot,
                rotationData,
                movementModeToggleCheck,
                coordinateSystemLock);

        // Handle returned data values here
        double moveSpeed = driverTeleopData.moveSpeed;
        movespeedout = moveSpeed;

        movementModeToggleCheck = driverTeleopData.movementModeToggleCheck;
        coordinateSystemLock = driverTeleopData.coordinateSystemLock;
        rotationLockAngle = driverTeleopData.rotationData.rotationLockAngle;
        leftRotateCoordCheck = driverTeleopData.rotationData.leftRotateCoordCheck;
        rightRotateCoordCheck = driverTeleopData.rotationData.rightRotateCoordCheck;

        // Update Diag Power
        Vector2 left_stick = new Vector2(gamepad1.left_stick_x, gamepad1.left_stick_y);
        double leftDiagPower = TeleopDriverControls.getLeftDiagPower(left_stick, robot, coordinateSystemLock, rotationLockAngle, telemetry);
        double rightDiagPower = TeleopDriverControls.getRightDiagPower(left_stick, robot, coordinateSystemLock, rotationLockAngle, telemetry);

        double leftRotatePower = gamepad1.right_stick_x;
        double rightRotatePower = -gamepad1.right_stick_x;

        tankControlsVeryFast = gamepad1.right_bumper;

        if (tankControlsVeryFast) {
            if (shouldRotateInPlace(gamepad1.right_stick_x)) {
                rotateInPlace(moveSpeed);
            } else {
                doTankControlls();
            }
        } else {
            // Update Robot Drive
            double frontLeftWheelPower = moveSpeed * (leftDiagPower + leftRotatePower);
            double frontRightWheelPower = moveSpeed * (rightDiagPower + rightRotatePower);
            leftfrontpowerOut = frontLeftWheelPower;
            double backLeftWheelPower = moveSpeed * (rightDiagPower + leftRotatePower);
            double backRightWheelPower = moveSpeed * (leftDiagPower + rightRotatePower);
            robot.updateRobotDrive(frontLeftWheelPower, frontRightWheelPower, backLeftWheelPower, backRightWheelPower);
        }

        //NEEDS TESTING BEFORE USE! DONT USE THIS IN COMP
        double overrideDeadzone = 0.1;

        if ((Math.sqrt(bMath.squared(gamepad1.left_stick_x) + (bMath.squared(gamepad1.left_stick_y))))
                < overrideDeadzone && Math.abs(gamepad1.right_stick_x) < 0.1) {
            robot.setPowerDouble4(0, 0, 0, 0, 0);
        }

    }

    private boolean shouldRotateInPlace(double rotatationControlValue) {
        double tolerance = 0.1;
        if (Math.abs(rotatationControlValue) > tolerance) {
            return true;
        } else {
            return false;
        }
    }

    private void rotateInPlace(double speed) {
        double leftRotatePower = gamepad1.right_stick_x;
        double rightRotatePower = -gamepad1.right_stick_x;

        double frontLeftWheelPower = speed * (leftRotatePower);
        double frontRightWheelPower = speed * (rightRotatePower);
        double backLeftWheelPower = speed * (leftRotatePower);
        double backRightWheelPower = speed * (rightRotatePower);
        robot.updateRobotDrive(frontLeftWheelPower, frontRightWheelPower, backLeftWheelPower, backRightWheelPower);
    }

    private void doTankControlls() {

        //if you're not pointing in a direction, stop
        if (Math.abs(gamepad1.left_stick_x) < 0.1 && Math.abs(gamepad1.left_stick_y) < 0.1) {
            robot.setWheelPowersInAClockwiseOrder(0, 0, 0, 0);
            return;
        }

        //find the angle the left joystick is pointing at (this joystick controls the direction of movement)
        double movementAngle = findDesiredAngleInRadians(gamepad1.left_stick_x, gamepad1.left_stick_y);

        //approxMovementAngle method might make it hard to move at 45°; you'll have to be precise with the controller
        int approxMovementAngle = snapAngleToAMultipleOf45degreesAndPutOutputInDegrees(movementAngle);

        //TODO understand why this is essentially right even though it doesn't make sense
        if (approxMovementAngle == 0) {
            robot.setWheelPowersInAClockwiseOrder(1, -1, 1, -1);
        }

        //these values look like they should make it go forwards, but that's not how it was behaving
        if (approxMovementAngle == 270) {
            robot.setWheelPowersInAClockwiseOrder(1, 1, 1, 1);
        }
        if (approxMovementAngle == 180) {
            robot.setWheelPowersInAClockwiseOrder(-1, 1, -1, 1);
        }

        //TODO fix this and make both it and 270 make sense
        if (approxMovementAngle == 90) {
            robot.setWheelPowersInAClockwiseOrder(-1, -1, -1, -1);
        }


        //this block of if statements handles it if it's snapped to a multiple of 45°
        if (approxMovementAngle == 45) {
            robot.setWheelPowersInAClockwiseOrder(1, 0, 1, 0);
        }
        if (approxMovementAngle == 135) {
            robot.setWheelPowersInAClockwiseOrder(0, 1, 0, 1);
        }
        if (approxMovementAngle == 225) {
            robot.setWheelPowersInAClockwiseOrder(-1, 0, -1, 0);
        }
        if (approxMovementAngle == 315) {
            robot.setWheelPowersInAClockwiseOrder(0, -1, 0, -1);
        }


    }

    private double findDesiredAngleInRadians(double xComponent, double yComponent) {
        double angleTangent = Math.atan(yComponent / xComponent);
        if (xComponent < 0) {
            angleTangent += Math.PI;
        }
        return angleTangent;
    }

    private int snapAngleToAMultipleOf45degreesAndPutOutputInDegrees(double inputAngleInRadians) {
        double pi = Math.PI;
        double cosine = Math.cos(inputAngleInRadians);
        double sine = Math.sin(inputAngleInRadians);
        //these 4 say that if it's close enough to a multiple of 90, go with that multiple
        if (cosine >= Math.cos(pi / 4)) {
            return 0;
        }
        if (sine >= Math.sin(pi / 4)) {
            return 90;
        }
        if (cosine <= Math.cos(3 * pi / 4)) {
            return 180;
        }
        if (sine < Math.sin(-1 * pi / 4)) {
            return 270;
        }

        /*
        UPDATE: the program should now never get below this point
            Explanation:
                The drivers said they never were going to use the option to go at 45°
                so now it shouldn't anymore
         */

        //if it's not that close, keep the unit circle point the same but make the angle between 0 and 2pi
        while (inputAngleInRadians < 0) {
            inputAngleInRadians += 2 * pi;
        }
        while (inputAngleInRadians >= 2 * pi) {
            inputAngleInRadians -= 2 * pi;
        }

        //then find the nearest multiple of 45° (pi / 4 radians)
        if (pi / 6 < inputAngleInRadians && inputAngleInRadians < pi / 3) {
            return 45;
        }
        if (2 * pi / 3 < inputAngleInRadians && inputAngleInRadians < 5 * pi / 6) {
            return 135;
        }
        if (7 * pi / 6 < inputAngleInRadians && inputAngleInRadians < 4 * pi / 3) {
            return 225;
        }
        if (5 * pi / 3 < inputAngleInRadians && inputAngleInRadians < 11 * pi / 6) {
            return 315;
        }

        //if none of that works, then something has gone horribly wrong
        snapAngle_functionWentHorriblyWrong = true;
        return 6000;
    }

    // ************** Arm Methods **************

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
    private void moveArm(Robot robot, EngineeringControlData engiData) {
        if (engiData.powerExtension) {
            robot.arm.SetArmStateExtensionPower(engiData.extendSpeed, engiData.raiseSpeed);
        } else {
            robot.arm.SetArmStatePower(engiData.extension, engiData.raiseSpeed);
        }
    }

    // ************** Servo Methods **************

    // TODO: - This Methods needs to be refactored and all of the related state needs to be consolidated

    private void updateServoControls() {

        //press the X button to put the grabber in "idle" position
        idle = TeleopServosControls.getUpdatedIdle(gamepad2, xButton2Check, idle);
        xButton2Check = gamepad2.x;

        // TODO: - Refactor this into TeleopServosControls
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


        gripAngle = TeleopServosControls.rotateGripperDown(gamepad2, gripAngle, deltaTime);
        gripAngle = TeleopServosControls.rotateGripperUp(gamepad2, gripAngle, deltaTime);
        gripAngle = TeleopServosControls.pointGripperDown(gamepad2, robot, gripAngle);


        // TODO: - Refactor these 2 into TeleopServosControls
        // move foundation grippers with b button
        if (gamepad1.b && !bButton1Check) {
            gripFoundation = !gripFoundation;
        }
        bButton1Check = gamepad1.b;

        // dropLunchBox with y button
        if (gamepad2.y && !yButton2Check) {
            dropLunchBox = !dropLunchBox;
        }
        yButton2Check = gamepad2.y;


        lunchboxRot = TeleopServosControls.getLunchBoxRot(dropLunchBox);
        engiData = TeleopServosControls.protectSpoolAndUpdateEngiData(gamepad2, engiData, robot);
        gripAngle = TeleopServosControls.moveServosAndGetGripAngle(robot, lunchboxRot, gripAngle, idle, grab, gripFoundation);
    }


    // ************** Telemetry **************

    private void doTelemetry(Telemetry telemetry,
                             ElapsedTime deltaTime,
                             boolean coordinateSystemLock,
                             EngineeringControlData engiData,
                             double lunchboxRot, boolean horriblyWrong) {
        telemetry.addData("deltaTime", deltaTime.milliseconds());
        telemetry.addData("Rotation Locked ", coordinateSystemLock);
        telemetry.addData("RectWanted?:", engiData.rectControls);
        telemetry.addData("spoolProtect", engiData.spoolProtect);
        telemetry.addData("Current Lunchbox", lunchboxRot);
        telemetry.addData("xExtConst", engiData.xExtConst);
        telemetry.addData("yExtConst", engiData.yExtConst);
        telemetry.addLine("-----Pot-----");
        telemetry.addData("potVoltage:", robot.armPotentiometer.getVoltage());
        telemetry.addData("potAngle:", bMath.toDegrees(robot.armPotentiometer.getAngle()));
        telemetry.addData("RegSlope:", robot.armPotentiometer.regSlope);
        telemetry.addData("RegIntercept:", robot.armPotentiometer.regIntercept);
        telemetry.addData("thetaAngle:", bMath.toDegrees(robot.arm.thetaAngle()));
        telemetry.addLine("-----WheelInfo-----");
        telemetry.addData("moveSpeed,", movespeedout);
        telemetry.addData("leftFront:", leftfrontpowerOut);
        telemetry.update();
    }
}
