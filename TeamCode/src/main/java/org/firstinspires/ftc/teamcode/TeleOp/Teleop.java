package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Hardware.bMotor;
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


    // ************** Life Cycle Methods **************
    @Override
    public void runOpMode() throws InterruptedException {
        preStartSetup();

        waitForStart();

        lunchboxRot = 1;
        robot.arm.setGripState(RobotArm.GripState.IDLE, 0);
//        robot.arm.setGripState(RobotArm.GripState.IDLE, 60);
        gripAngle = 180;

        while (opModeIsActive()) {
            updateDriverControls();
            updateArm();
            updateServoControls();
            doTelemetry();
            deltaTime.reset(); // Update deltaTime
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

        movementModeToggleCheck = driverTeleopData.movementModeToggleCheck;
        coordinateSystemLock = driverTeleopData.coordinateSystemLock;
        rotationLockAngle = driverTeleopData.rotationData.rotationLockAngle;
        leftRotateCoordCheck = driverTeleopData.rotationData.leftRotateCoordCheck;
        rightRotateCoordCheck = driverTeleopData.rotationData.rightRotateCoordCheck;

        // Update Diag Power
        double leftDiagPower = TeleopDriverControls.getLeftDiagPower(gamepad1, robot, coordinateSystemLock, rotationLockAngle, telemetry);
        double rightDiagPower = TeleopDriverControls.getRightDiagPower(gamepad1, robot, coordinateSystemLock, rotationLockAngle, telemetry);

        double leftRotatePower = gamepad1.right_stick_x;
        double rightRotatePower = -gamepad1.right_stick_x;

        // Update Robot Drive
        double frontLeftWheelPower = moveSpeed * (leftDiagPower + leftRotatePower);
        double frontRightWheelPower = moveSpeed * (rightDiagPower + rightRotatePower);
        double backLeftWheelPower = moveSpeed * (rightDiagPower + leftRotatePower);
        double backRightWheelPower = moveSpeed * (leftDiagPower + rightRotatePower);
        robot.updateRobotDrive(frontLeftWheelPower, frontRightWheelPower, backLeftWheelPower, backRightWheelPower);
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
        } else  {
            robot.arm.SetArmStatePower(engiData.extension, engiData.raiseSpeed);
        }
    }

    /*
    Servo
     */

    // TODO: - This Methods needs to be refactored and all of the related state needs to be consolidated

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


        gripAngle = TeleopServosControls.pointGripperDown(gamepad2, robot, gripAngle);
        gripAngle = TeleopServosControls.rotateGripperDown(gamepad2, gripAngle, deltaTime);
        gripAngle = TeleopServosControls.rotateGripperUp(gamepad2, gripAngle, deltaTime);


        // Upon second look, these 2 bugs may not be bugs, but it's bad practice to code this way as the code intention is very difficult to comprehend

        // ??? Bug... due to missing {}
//        //move foundation grippers with b button
//        if (gamepad1.b && !bButton1Check) gripFoundation = !gripFoundation;
//        bButton1Check = gamepad1.b;

        //move foundation grippers with b button
        if (gamepad1.b && !bButton1Check) {
            gripFoundation = !gripFoundation;
            bButton1Check = gamepad1.b; // Also, BUG? Should this be gamepad 1 ???
        }

        // ??? Bug... due to missing {}
//        if (gamepad2.y && !yButton2Check) dropLunchBox = !dropLunchBox;
//        yButton2Check = gamepad2.y;

        if (gamepad2.y && !yButton2Check) {
            dropLunchBox = !dropLunchBox;
            yButton2Check = gamepad2.y;
        }

        lunchboxRot = TeleopServosControls.getLunchBoxRot(dropLunchBox);

        engiData = TeleopServosControls.protectSpoolAndUpdateEngiData(gamepad2, engiData, robot);

        gripAngle = TeleopServosControls.moveServosAndGetGripAngle(robot, lunchboxRot, gripAngle, idle, grab, gripFoundation);
    }


        // ************** Telementry **************

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