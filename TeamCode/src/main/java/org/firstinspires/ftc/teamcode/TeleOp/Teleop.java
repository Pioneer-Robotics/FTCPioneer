package org.firstinspires.ftc.teamcode.TeleOp;


import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Helpers.Vector2;
import org.firstinspires.ftc.teamcode.Helpers.bMath;
import org.firstinspires.ftc.teamcode.Robot.Robot;
import org.firstinspires.ftc.teamcode.Robot.RobotArm;
import org.firstinspires.ftc.teamcode.Robot.RobotConfiguration;

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
    private double ogLeftDiagPower = 0;
    private double ogRightDiagPower = 0;
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
    private boolean rectControls = false;
    private boolean rectControlsCheck = false;
    private boolean rectControls_goingUp = false;
    private boolean rectControls_goingUpCheck = false;

    //Gripper Control
    private boolean grab = false; //whether the gripper is gripping
    private boolean bButton2Check = false; //prevState of grab

    private boolean idle = false; //whether the gripper is in rest position
    private boolean xButton2Check = false;

    private boolean dropLunchBox = false;
    private boolean yButton2Check = false;

    private boolean pointDown = false;
    private boolean aButton2Check = false;

    private boolean fineServoControl = true;

    private double lunchboxRot = 0.5;

    private boolean gripFoundation = false;
    private boolean bButton1Check = false;
    //Mode Switch Variables


    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(this, false);
        fineServoControl = true;

        initControllers();
        waitForStart();

        lunchboxRot = 1;
        robot.arm.SetGripState(RobotArm.GripState.IDLE, 60);
        gripAngle = 30;
        while (opModeIsActive()) {
            telemetry.addLine("------ Control  ------");
            telemetry.addData("spool position", robot.arm.length.getCurrentPosition());
            telemetry.addData("spool position as percent", robot.arm.length.getCurrentPosition() / RobotConfiguration.arm_lengthMax);
            telemetry.addData("arm rotation as percent", robot.arm.rotation.getCurrentPosition() / RobotConfiguration.arm_rotationMax);


            ///DRIVER CONTROLS
            setupDriverController();

            getRightDiagPower(true, gamepad1.left_stick_x, gamepad1.left_stick_y, 90, 90);

            leftRotatePower = gamepad1.right_stick_x;
            rightRotatePower = -gamepad1.right_stick_x;

            double frontLeftWheelPower = moveSpeed * (leftDiagPower + leftRotatePower);
            double frontRightWheelPower = moveSpeed * (rightDiagPower + rightRotatePower);
            double backLeftWheelPower = moveSpeed * (rightDiagPower + leftRotatePower);
            double backRightWheelPower = moveSpeed * (leftDiagPower + rightRotatePower);
            updateRobotDrive(frontLeftWheelPower, frontRightWheelPower, backLeftWheelPower, backRightWheelPower);


            //ARM CONTROLS//

            // Activates rectControls when right stick is being moved
            rectControls = ((Math.abs(gamepad2.right_stick_y) > 0.1) || (Math.abs(gamepad2.right_stick_x) > 0.1));
            //sets direction of rectControls to whichever axis is greater
            rectControls_goingUp = Math.abs(gamepad2.right_stick_y) > Math.abs(gamepad2.right_stick_x);

            //get new extension constants if rectControls changes or if direction changes
            if ((rectControls != rectControlsCheck) || (rectControls_goingUp != rectControls_goingUpCheck))
                robot.arm.ExtConstCalc();
            rectControlsCheck = rectControls;
            rectControls_goingUpCheck = rectControls_goingUp;


            //Allows the Gripper to be moved straight up and down with the right joystick
            if (rectControls) {
                telemetry.addLine("Arm Control: Rect");
                //set power and distance to the Arm.
                robot.arm.SetArmStatePowerCm(robot.arm.RectExtension(rectControls_goingUp),
                        rectControls_goingUp ? gamepad2.right_stick_y : -gamepad2.right_stick_x);
                extension = robot.arm.cmToTicks(robot.arm.RectExtension(rectControls_goingUp)) / RobotConfiguration.arm_lengthMax;
            } else {
                telemetry.addLine("Arm Control: Radial");

                extension += gamepad2.right_trigger * deltaTime.seconds() * 1;    //extend arm when right trigger held
                extension -= gamepad2.left_trigger * deltaTime.seconds() * 1;     //retra ct arm when left trigger held

                raiseSpeed = bMath.Clamp(gamepad2.left_stick_y, -1, 1);
                extension = bMath.Clamp(extension, 0, 1);
                robot.arm.SetArmStatePower(extension, raiseSpeed);
            }

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
                robot.arm.SetGripState(RobotArm.GripState.IDLE, gripAngle / 180);
            } else if (grab) {
                robot.arm.SetGripState(RobotArm.GripState.CLOSED, gripAngle / 180);
            } else {
                robot.arm.SetGripState(RobotArm.GripState.OPEN, gripAngle / 180);
            }


            //press a button to make the gripper point down
            if (gamepad2.a && !aButton2Check) {
                pointDown = true;
            }
            aButton2Check = gamepad2.a;

            if (pointDown) {
                gripAngle = 90 - robot.arm.thetaAngle();
            }

            //rotate gripper down with the left dpad
            if (gamepad2.left_bumper) {
                gripAngle += deltaTime.seconds() * 135 * 1.5;
                pointDown = false;
            }

            //rotate gripper up with the right dpad
            if (gamepad2.right_bumper) {
                gripAngle -= deltaTime.seconds() * 135 * 1.5;
                pointDown = false;
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



            telemetry.addLine("------ Movement ------");
            telemetry.addData("Rotation Locked ", coordinateSystemLock);
            telemetry.addData("Current Rotation ", robot.getRotation());
//            telemetry.addData("Offset Angle ", angle);
            telemetry.addLine("-------- Arm  --------");
            telemetry.addData("Current Arm Angle", bMath.toDegrees(robot.arm.thetaAngle()));
            telemetry.addData("Current Potentiometer angle", robot.armPotentiometer.getAngle());
            telemetry.addData("Current Potentiometer voltage", robot.armPotentiometer.getVoltage());
            telemetry.addData("RectWanted?:", rectControls);
            telemetry.addData("ExtensionCurrent", robot.arm.rotation.getCurrentPosition());
            telemetry.addData("ExtensionWanted", extension);
            telemetry.addLine("------ Lunchbox ------");
            telemetry.addData("Current Lunchbox", lunchboxRot);
            telemetry.update();

            deltaTime.reset();
        }
        robot.shutdown();
    }

    private double fullRotation = 360;

    private void updateRobotDrive(double frontLeft, double frontRight, double backLeft, double backRight) {
        robot.driveManager.frontLeft.setPower(frontLeft);
        robot.driveManager.frontRight.setPower(frontRight);
        robot.driveManager.backLeft.setPower(backLeft);
        robot.driveManager.backRight.setPower(backRight);
    }

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


    private Vector2 getMovementVector(double movementInput_x, double movementInput_y) {
        double angle = Math.toRadians(robot.getRotation() - rotationLockAngle);

        double movementSpeed = (Math.sqrt(Math.pow(movementInput_x, 2) + Math.pow(movementInput_y, 2)));

        double _newGamepadX = movementSpeed * Math.cos(angle + Math.atan(movementInput_y / movementInput_x));
        double _newGamepadY = movementSpeed * Math.sin(angle + Math.atan(movementInput_y / movementInput_x));

        double newGamepadX = (gamepad1.left_stick_x <= 0) ? -_newGamepadX : _newGamepadX;
        double newGamepadY = (gamepad1.left_stick_x <= 0) ? -_newGamepadY : _newGamepadY;


        return new Vector2(newGamepadX, newGamepadY);
    }

    private void setupDriverController() {

        updateBoostorSlowMotion(gamepad1);

        // reset the "front" of the robot to be the real front
        if (gamepad1.a) {
            rotationLockAngle = robot.getRotation();
        }
        // up/down of right stick can incrementally change which way is the "front" of the robot in coordinate lock mode
        rotationLockAngle = ((rotationLockAngle + 3.0 * gamepad1.right_stick_y + fullRotation) % fullRotation) - fullRotation;


        updateDPad();

        // y button toggles coordinate system lock
        if (gamepad1.y && !movementModeToggleCheck) {
            rotationLockAngle = robot.getRotation();
            coordinateSystemLock = !coordinateSystemLock;
        }
        movementModeToggleCheck = gamepad1.y;

    }

    private void updateBoostorSlowMotion(Gamepad gamePad) {
        //let left bumper toggle boost vs slow mode on the right trigger for fine control of the robot
        if (!gamePad.left_bumper) {
            //trigger makes robot slower
            moveSpeed = bMath.Clamp(0.25 * (0.5 * (1 - gamepad1.right_trigger) + (1 - gamepad1.left_trigger) + 0.5), 0, 1);
        } else {
            //trigger makes robot faster
            moveSpeed = bMath.Clamp(0.5 + gamepad1.right_trigger / 2, 0, 1);
        }

    }

    private void updateDPad() {
        //left and right dpad can shift "front" of robot by 90 degrees in coordinate lock mode
        if (gamepad1.dpad_left && !leftRotateCoordCheck) {
            rotationLockAngle = ((rotationLockAngle + 450) % fullRotation) - fullRotation;
        }
        leftRotateCoordCheck = gamepad1.dpad_left;


        if (gamepad1.dpad_right && !rightRotateCoordCheck) {
            rotationLockAngle = ((rotationLockAngle + 270) % fullRotation) - fullRotation;
        }
        rightRotateCoordCheck = gamepad1.dpad_right;

    }


}