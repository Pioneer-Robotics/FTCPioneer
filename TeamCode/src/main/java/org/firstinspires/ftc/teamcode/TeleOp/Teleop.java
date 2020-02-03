package org.firstinspires.ftc.teamcode.TeleOp;


import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Helpers.Vector2;
import org.firstinspires.ftc.teamcode.Helpers.bMath;
import org.firstinspires.ftc.teamcode.Robot.Robot;
import org.firstinspires.ftc.teamcode.Robot.RobotArm;
import org.firstinspires.ftc.teamcode.Robot.RobotConfiguration;
import org.firstinspires.ftc.teamcode.TeleOp.DriverControls.DriverTeleopData;
import org.firstinspires.ftc.teamcode.TeleOp.DriverControls.RectangularControlData;
import org.firstinspires.ftc.teamcode.TeleOp.DriverControls.RotationData;
import org.firstinspires.ftc.teamcode.TeleOp.DriverControls.TeleopDriverControls;

/// Advice-
/// Consolidate State variable into a similar concept object. (see RectangularControlData for example etc...)
/// Move helper functions into Classes that relate (see Robot updateRobotDrive() for example, getMovementVector() still needs to be moved to a Math class)
/// Create layers of abstration to the code that makes it easier to read and understand (see TeleopDriverControls for example)
/// In general, attempt to create sub-layers of abstractions that have code that will almost never change so that if can be added to the code base and then confidently ignored as working flawlessly. (Adding self testing code with a testing framework would also be very helpful)
/// Adding layers of abstrations allow the coder to only focus on the level of code that needs to be updated (or read and understood) without unnecessary added complexity.


@TeleOp(name = "TeleOp", group = "Sensor")
public class Teleop extends TeleOpMode {


    private Robot robot = new Robot();

    private ElapsedTime deltaTime = new ElapsedTime();

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
    private double raiseSpeed = 0;
    private double extension = 0;
    private double gripAngle = 180;

    //Rectangular Control Variables New
    /// Code clean up advice --- State variable that have a similar concept should be grouped into a single object
    private RectangularControlData rectControlData = new RectangularControlData();

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

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(this, false);

        initControllers();
        waitForStart();

        lunchboxRot = 1;
        robot.arm.setGripState(RobotArm.GripState.IDLE, 60);
        gripAngle = 180;
        while (opModeIsActive()) {
            telemetry.addLine("------ Control  ------");

            updateDriverControls();

            //ARM CONTROLS//

            // TODO - Create `updateArmControls()` method
            // `updateArmControls()` should have leveled abstrations that make reading and understanding the code simpler.
            // Lower levels of the abstrations should be stashed in other classes.
            // The lower level abstrations should not store state if possible. They should be passed in values and then return values so that state can be easily be tracked.


        updateArmControls();



            //Gripper Controls//

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


            if (idle) {
                robot.arm.setGripState(RobotArm.GripState.IDLE, gripAngle / 180);
            } else if (grab) {
                robot.arm.setGripState(RobotArm.GripState.CLOSED, gripAngle / 180);
            } else {
                robot.arm.setGripState(RobotArm.GripState.OPEN, gripAngle / 180);
            }


            //hold A button to make the gripper point down
            if (gamepad2.a) {
                gripAngle = 90 - robot.arm.thetaAngle();
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
            robot.capstoneServo.setPosition(lunchboxRot);

            robot.foundationServo0.setPosition(gripFoundation ? 0.05 : 1);
            robot.foundationServo1.setPosition(gripFoundation ? 0.95 : 0);

            gripAngle = bMath.Clamp(gripAngle, 0, 180);

            // Print Updated Values to telemetry
            telemetry.addLine("------ Movement ------");
            telemetry.addData("Rotation Locked ", coordinateSystemLock);
            telemetry.addData("Current Rotation ", robot.getRotation());
//            telemetry.addData("Offset Angle ", angle);
            telemetry.addLine("-------- Arm  --------");
            telemetry.addData("Current Arm Angle", bMath.toDegrees(robot.arm.thetaAngle()));
            telemetry.addData("Current Potentiometer angle", robot.armPotentiometer.getAngle());
            telemetry.addData("RectWanted?:", rectControlData.rectControls);
            telemetry.addData("target spool position", extension*RobotConfiguration.arm_ticksMax);
            telemetry.addData("spool position", robot.arm.length.getCurrentPosition());
            telemetry.addData("spool position as percent", robot.arm.length.getCurrentPosition() / RobotConfiguration.arm_ticksMax);
            telemetry.addData("arm rotation as percent", robot.arm.rotation.getCurrentPosition() / RobotConfiguration.arm_rotationMax);
            telemetry.addLine("------ Lunchbox ------");
            telemetry.addData("Current Lunchbox", lunchboxRot);
            telemetry.update();

            // Update deltaTime
            deltaTime.reset();
        }
        robot.shutdown();
    }

    private void updateDriverControls() {
        ///DRIVER CONTROLS

        // Create driverTeleopData to get values to update robot state
        RotationData rotationData = new RotationData(rotationLockAngle, leftRotateCoordCheck, rightRotateCoordCheck);
        DriverTeleopData driverTeleopData = TeleopDriverControls.setupDriverController(gamepad1,
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
        double frontLeftWheelPower = moveSpeed * (leftDiagPower + leftRotatePower);
        double frontRightWheelPower = moveSpeed * (rightDiagPower + rightRotatePower);
        double backLeftWheelPower = moveSpeed * (rightDiagPower + leftRotatePower);
        double backRightWheelPower = moveSpeed * (leftDiagPower + rightRotatePower);
        robot.updateRobotDrive(frontLeftWheelPower, frontRightWheelPower, backLeftWheelPower, backRightWheelPower);
    }

    // TODO: - Move to ??? Class
    private double getLeftDiagPower(boolean useLockedRotation,
                                    double movementInput_x,
                                    double movementInput_y,
                                    double currentRobotRotation,
                                    double targetRotation) {
        // drive code
        if (useLockedRotation) {
            telemetry.addData("Drive System", "New");

            Vector2 movementVector = getMovementVector(movementInput_x, movementInput_y);
            return ((-movementVector.y + movementVector.x) / sq2);
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

            Vector2 movementVector = getMovementVector(movementInput_x, movementInput_y);
            return ((-movementVector.y - movementVector.x) / sq2);

        } else {
            telemetry.addData("Drive System", "Old");
            return ((-gamepad1.left_stick_y - gamepad1.left_stick_x) / sq2);
        }
    }

    // TODO: - Move to a Math Class
    private Vector2 getMovementVector(double movementInput_x, double movementInput_y) {
        double angle = Math.toRadians(robot.getRotation() - rotationLockAngle);

        double movementSpeed = (Math.sqrt(Math.pow(movementInput_x, 2) + Math.pow(movementInput_y, 2)));

        double _newGamepadX = movementSpeed * Math.cos(angle + Math.atan(movementInput_y / movementInput_x));
        double _newGamepadY = movementSpeed * Math.sin(angle + Math.atan(movementInput_y / movementInput_x));

        double newGamepadX = (gamepad1.left_stick_x <= 0) ? -_newGamepadX : _newGamepadX;
        double newGamepadY = (gamepad1.left_stick_x <= 0) ? -_newGamepadY : _newGamepadY;


        return new Vector2(newGamepadX, newGamepadY);
    }

    private void updateArmControls(){

        updateRectControls();

        //Allows the Gripper to be moved straight up and down with the right joystick
        if (rectControlData.rectControls) {
            telemetry.addLine("Arm Control: Rect");
            //set power and distance to the Arm.
            extension = bMath.Clamp(robot.arm.cmToTicks(
                            robot.arm.RectExtension(rectControlData.rectControls_goingUp,rectControlData.xExtConst,rectControlData.yExtConst))
                    / RobotConfiguration.arm_ticksMax);
            robot.arm.SetArmStatePower
                    (extension,
                            rectControlData.rectControls_goingUp ? -0.5*gamepad2.right_stick_y : -0.5*gamepad2.right_stick_x);
        } else {
            telemetry.addLine("Arm Control: Radial");

            extension += gamepad2.right_trigger * deltaTime.seconds() * 1.5;    //extend arm when right trigger held
            extension -= gamepad2.left_trigger * deltaTime.seconds() * 1.5;     //retract arm when left trigger held

            raiseSpeed = bMath.Clamp(-gamepad2.left_stick_y, -1, 1);
            extension = bMath.Clamp(extension, 0, 1);
            robot.arm.SetArmStatePower(extension, raiseSpeed);
        }
    }

    private void updateRectControls(){
        // Activates rectControls when right stick is being moved
        rectControlData.rectControls = ((Math.abs(gamepad2.right_stick_y) > 0.1) || (Math.abs(gamepad2.right_stick_x) > 0.1));
        //sets direction of rectControls to whichever axis is greater
        rectControlData.rectControls_goingUp = Math.abs(gamepad2.right_stick_y) > Math.abs(gamepad2.right_stick_x);

        //get new extension constants if rectControls changes or if direction changes
        if ((rectControlData.rectControls != rectControlData.rectControlsCheck) || (rectControlData.rectControls_goingUp != rectControlData.rectControls_goingUpCheck))
            rectControlData.xExtConst =  robot.arm.xExtConst();
        rectControlData.yExtConst = robot.arm.yExtConst();

        rectControlData.rectControlsCheck = rectControlData.rectControls;
        rectControlData.rectControls_goingUpCheck = rectControlData.rectControls_goingUp;
    }







}

