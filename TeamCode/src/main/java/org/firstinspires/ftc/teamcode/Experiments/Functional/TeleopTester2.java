package org.firstinspires.ftc.teamcode.Experiments.Functional;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Helpers.bMath;
import org.firstinspires.ftc.teamcode.Robot.Robot;
import org.firstinspires.ftc.teamcode.Robot.RobotArm;
import org.firstinspires.ftc.teamcode.Robot.RobotConfiguration;

@TeleOp(name = "Teleop2", group = "Sensor")
public class TeleopTester2 extends LinearOpMode {


    Robot robot = new Robot();

    ElapsedTime deltaTime = new ElapsedTime();

    //Program State

    //Driver Control Variables
    private double moveSpeed;

    private boolean leftRotateCoordCheck = false;
    private boolean rightRotateCoordCheck = false;

    private double angle = 0;
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

    private double newGamepadX;
    private double newGamepadY;

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
        robot.init(hardwareMap, this, false);
        fineServoControl = true;


        waitForStart();

        lunchboxRot = 1;
        robot.arm.SetGripState(RobotArm.GripState.IDLE, 60);
        gripAngle = 30;
        while (opModeIsActive()) {
            telemetry.addLine("------ Control  ------");

            ///DRIVER CONTROLS
            setupDriverController();

            // drive code
            if (coordinateSystemLock) {
                telemetry.addData("Drive System", "New");

                angle = Math.toRadians(robot.GetRotation() - rotationLockAngle);

                /*
                EXAMPLE of the old (and better) math

                first the original diag powers are used
                ogLeftDiagPower = ((-gamepad1.left_stick_y + gamepad1.left_stick_x) / sq2);
                ogRightDiagPower = ((-gamepad1.left_stick_y - gamepad1.left_stick_x) / sq2);

                then we use use the coordinate rotation formula (see https://en.wikipedia.org/wiki/Rotation_of_axes#Derivation) to get the new coordinates

                leftDiagPower = ogLeftDiagPower*Math.cos(angle)+ogRightDiagPower*Math.sin(angle);
                rightDiagPower = -ogLeftDiagPower*Math.sin(angle)+ogRightDiagPower*Math.cos(angle);

                Finally we compose the first two equations into the second two to get the formulas you see below. This is simpler and requires less computation than Josh's method (I use 2 trig and 0 square root calculations compared to josh using 4 trig and 2 sqrt, not to mention he also
                has conditionals), the problem bust be somewhere else in the code, either with entering the lock state or the computation of the offset angle that is put into the formula
                 */
                //leftDiagPower = ((-gamepad1.left_stick_y + gamepad1.left_stick_x) / sq2) * Math.cos(angle))+((-gamepad1.left_stick_y - gamepad1.left_stick_x) / sq2 * Math.sin(angle);
                //rightDiagPower = (((-(-gamepad1.left_stick_y + gamepad1.left_stick_x) / sq2) * Math.sin(angle)) + (((-gamepad1.left_stick_y - gamepad1.left_stick_x) / sq2 * Math.cos(angle))));
////              This is Josh's idea that may work if any of the above doesn't (He's pretty confidant about it.... He tested it in Desmos and everything). The operative idea below is that it just adjusts the input vector from the gamepad to one that has the same magnitude but an angle
////              that is equal to (originalAngle + angle) which I think then would make it move in the desired direction in real life. Then it just uses the movement code from the nonlock version to move along that new input vector
//
                newGamepadX = (Math.sqrt(Math.pow(gamepad1.left_stick_x, 2) + Math.pow(gamepad1.left_stick_y, 2))) * Math.cos(angle + Math.atan(gamepad1.left_stick_y / gamepad1.left_stick_x));
                newGamepadY = (Math.sqrt(Math.pow(gamepad1.left_stick_x, 2) + Math.pow(gamepad1.left_stick_y, 2))) * Math.sin(angle + Math.atan(gamepad1.left_stick_y / gamepad1.left_stick_x));
//
////              The following two lines just correct the signs of newGamePadX and newGamePadY because the domain of the Math.atan function will make this always return angles in Q1 and Q4, so when the point is in Q2 and Q3 then you gotta adjust it
                newGamepadX = (gamepad1.left_stick_x <= 0) ? -newGamepadX : newGamepadX;
                newGamepadY = (gamepad1.left_stick_x <= 0) ? -newGamepadY : newGamepadY;
                leftDiagPower = ((-newGamepadY + newGamepadX) / sq2);
                rightDiagPower = ((-newGamepadY - newGamepadX) / sq2);
////              //And so Ends Josh's Idea.

                //this could replace the lines above and the 6 lines after the else but the implementation in this function
                //robot.MoveComplex(new Double2(gamepad1.left_stick_x,gamepad1.left_stick_y),moveSpeed,gamepad1.right_stick_x,angle);
            } else {
                telemetry.addData("Drive System", "Old");

                leftDiagPower = ((-gamepad1.left_stick_y + gamepad1.left_stick_x) / sq2);
                rightDiagPower = ((-gamepad1.left_stick_y - gamepad1.left_stick_x) / sq2);

                //robot.MoveComplex(new Double2(gamepad1.left_stick_x,gamepad1.left_stick_y),moveSpeed,gamepad1.right_stick_x,0);

            }
            leftRotatePower = gamepad1.right_stick_x;
            rightRotatePower = -gamepad1.right_stick_x;
            robot.driveManager.frontLeft.setPower(moveSpeed * (leftDiagPower + leftRotatePower));
            robot.driveManager.frontRight.setPower(moveSpeed * (rightDiagPower + rightRotatePower));
            robot.driveManager.backLeft.setPower(moveSpeed * (rightDiagPower + leftRotatePower));
            robot.driveManager.backRight.setPower(moveSpeed * (leftDiagPower + rightRotatePower));

            //The following is used for recalibrating the lunchbox servo
            /*
            //rotate lunchbox up with the up dpad
            lunchboxRot -= gamepad1.dpad_up ? deltaTime.seconds() * 1 : 0;
            //rotate lunchbox down with the down dpad
            lunchboxRot += gamepad1.dpad_down ? deltaTime.seconds() * 1 : 0;
            lunchboxRot = bMath.Clamp(lunchboxRot, 0, 1);
            robot.lunchBox.setPosition(lunchboxRot);
*/


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
                extension = robot.arm.CmToTicks(robot.arm.RectExtension(rectControls_goingUp)) / RobotConfiguration.arm_ticksMax;
            } else {
                telemetry.addLine("Arm Control: Radial");

                extension += gamepad2.right_trigger * deltaTime.seconds() * 2;    //extend arm when right trigger held
                extension -= gamepad2.left_trigger * deltaTime.seconds() * 2;     //retra ct arm when left trigger held

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
            if (gamepad2.dpad_left) {
                gripAngle += deltaTime.seconds() * 135;
                pointDown = false;
            }

            //rotate gripper up with the right dpad
            if (gamepad2.dpad_right) {
                gripAngle -= deltaTime.seconds() * 135;
                pointDown = false;
            }

            //move foundation grippers with b button
            if (gamepad1.b && !bButton1Check) gripFoundation = !gripFoundation;
            bButton1Check = gamepad1.b;

            if (gamepad2.y && !yButton2Check) dropLunchBox = !dropLunchBox;
            yButton2Check = gamepad2.y;

            if (dropLunchBox) lunchboxRot = 0;
            else lunchboxRot = 0.738;
            robot.lunchBox.setPosition(lunchboxRot);

            robot.foundationServo0.setPosition(gripFoundation ? 0 : 1);
            robot.foundationServo1.setPosition(gripFoundation ? 1 : 0);

            gripAngle = bMath.Clamp(gripAngle, 0, 180);


            telemetry.addLine("------ Movement ------");
            telemetry.addData("Rotation Locked ", coordinateSystemLock);
            telemetry.addData("Current Rotation ", robot.GetRotation());
            telemetry.addData("Offset Angle ", angle);
            telemetry.addLine("-------- Arm  --------");
            telemetry.addData("Current Arm Angle", robot.arm.thetaAngle());
            telemetry.addData("Current Potentiometer value", robot.armPotentiometer.getAngle());
            telemetry.addData("Interior Angle", robot.armPotentiometer.getAngle() + RobotConfiguration.pot_interiorOffset);
            telemetry.addData("RectWanted?:", rectControls);
            telemetry.addLine("------ Lunchbox ------");
            telemetry.addData("Current Lunchbox", lunchboxRot);
            telemetry.update();

            deltaTime.reset();
        }
        robot.Stop();
    }

    double fullRotation = 360;


    private void setupDriverController() {

        updateBoostorSlowMotion(gamepad1);

        // reset the "front" of the robot to be the real front
        if (gamepad1.a) {
            rotationLockAngle = robot.GetRotation();
        }
        // up/down of right stick can incrementally change which way is the "front" of the robot in coordinate lock mode
        rotationLockAngle = ((rotationLockAngle + 3.0 * gamepad1.right_stick_y + fullRotation) % fullRotation) - fullRotation;


        updateDPad();

        // y button toggles coordinate system lock
        if (gamepad1.y && !movementModeToggleCheck) {
            rotationLockAngle = robot.GetRotation();
            coordinateSystemLock = !coordinateSystemLock;
        }
        movementModeToggleCheck = gamepad1.y;

    }

    private void updateBoostorSlowMotion(Gamepad gamePad) {
        //let left bumper toggle boost vs slow mode on the right trigger for fine control of the robot
        if (gamePad.left_bumper) {
            //trigger makes robot slower
            moveSpeed = bMath.Clamp(0.5 - gamepad1.right_trigger / 2, 0, 1);
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