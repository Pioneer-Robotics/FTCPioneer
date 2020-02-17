package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Hardware.bMotor;
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
    private double leftfrontpower = 0;

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
            doTelemetry(telemetry, deltaTime, coordinateSystemLock, engiData, lunchboxRot);
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
        movespeedout = moveSpeed;

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
        leftfrontpower = frontLeftWheelPower;
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
                                    double lunchboxRot) {
        telemetry.addData("deltaTime", deltaTime.milliseconds());
        telemetry.addData("Rotation Locked ", coordinateSystemLock);
        telemetry.addData("RectWanted?:", engiData.rectControls);
        telemetry.addData("spoolProtect", engiData.spoolProtect);
        telemetry.addData("Current Lunchbox", lunchboxRot);
        telemetry.addData("xExtConst",engiData.xExtConst);
        telemetry.addData("yExtConst",engiData.yExtConst);
        telemetry.addLine("-----Pot-----");
        telemetry.addData("potVoltage:", robot.armPotentiometer.getVoltage());
        telemetry.addData("potAngle:", bMath.toDegrees(robot.armPotentiometer.getAngle()));
        telemetry.addData("RegSlope:", robot.armPotentiometer.regSlope);
        telemetry.addData("RegIntercept:", robot.armPotentiometer.regIntercept);
        telemetry.addData("thetaAngle:", bMath.toDegrees(robot.arm.thetaAngle()));
        telemetry.addLine("-----WheelInfo-----");
        telemetry.addData("moveSpeed,", movespeedout);
        telemetry.addData("leftFront:", leftfrontpower);
        telemetry.update();
    }
}