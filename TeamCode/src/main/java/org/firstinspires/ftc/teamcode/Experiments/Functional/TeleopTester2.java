package org.firstinspires.ftc.teamcode.Experiments.Functional;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Helpers.bMath;
import org.firstinspires.ftc.teamcode.Robot.Robot;
import org.firstinspires.ftc.teamcode.Robot.RobotArm;
import org.firstinspires.ftc.teamcode.Robot.RobotWallTrack;

@TeleOp(name = "Teleop2", group = "Sensor")
public class TeleopTester2 extends LinearOpMode {

    RobotWallTrack.SensorGroup targetWallTrackGroup = null;


    Robot robot = new Robot();

    ElapsedTime deltaTime = new ElapsedTime();

    //Program State
    
    //Driver Control Variables
    double moveSpeed;
    //Arm Control Variables
    double raiseSpeed = 0;
    double extension = 0;
    double armAngle = 0;
    double gripAngle = 180;
    //Rectangular Control Variables New
    
    //Rectangular Control Variables Old
    double yWanted = 0;
    double xWanted = 0;
    double vertDMove = 0;
    
    //Gripper Control
    boolean grab = false; //whether the gripper is gripping
    boolean bButton2Check = false; //prevState of grab
    
    boolean idle = false; //whether the gripper is in rest position
    boolean xButton2Check = false;
    
    boolean pointDown = false;
    boolean aButton2Check = false;
    
    //Mode Switch Variables
    
    


    

    
    boolean lastD2press = false;
    boolean leftRotateCoordCheck = false;
    boolean rightRotateCoordCheck = false;

    double angle = 0;
    double leftDiagPower = 0;
    double rightDiagPower = 0;
    final double sq2 = Math.pow(2, 1/2);
    double leftRotatePower = 0;
    double rightRotatePower = 0;



    boolean rectControls = false;
    boolean rectControlsCheck = false;
    boolean rectControls_goingUp = false;
    boolean rectControls_goingUpCheck = false;

    boolean leftBumper2Check = false;
    double targetGripperPositionY = 0;
    double targetGripperPositionX = 0;

    boolean movementModeToggleCheck = false;
    boolean coordinateSystemLock = false;


    double rotationLockAngle = 0;

    double lunchboxRot = 0.5;

    boolean servoLastToggle = false;
    boolean fineServoControl = true;

    boolean gripFoundation = false;
    boolean bLast = false;


    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap, this);
        fineServoControl = true;


        waitForStart();

        lunchboxRot = 1;
        robot.arm.SetGripState(RobotArm.GripState.IDLE, 60);
        gripAngle = 30;
        while (opModeIsActive()) {

            ///DRIVER CONTROLS

            //let left bumper put robot in really slow mode for fine control
            if (gamepad1.left_bumper) {
                //trigger makes robot slower
                moveSpeed = bMath.Clamp(0.5 - gamepad1.right_trigger/2, 0, 1);
            } else {
                //trigger makes robot faster
                moveSpeed = bMath.Clamp(0.5 + gamepad1.right_trigger/2, 0, 1);
            }
            // reset the "front" of the robot to be the real front
            if (gamepad1.a) {
                rotationLockAngle = robot.GetRotation();
            }
            // up/down of right stick can incrementally change which way is the "front" of the robot in coordinate lock mode
            rotationLockAngle = ((rotationLockAngle + 3.0*gamepad1.right_stick_y+360)%360)-360;

            //left and right dpad can shift "front" of robot by 90 degrees in coordinate lock mode
            if (gamepad1.dpad_left && !leftRotateCoordCheck) {
                rotationLockAngle = ((rotationLockAngle + 450)%360)-360;
            }
            leftRotateCoordCheck = gamepad1.dpad_left;
            if (gamepad1.dpad_right && !rightRotateCoordCheck) {
                rotationLockAngle = ((rotationLockAngle + 270)%360)-360;
            }
            rightRotateCoordCheck = gamepad1.dpad_right;

            // y button toggles coordinate system lock
            if (gamepad1.y && !movementModeToggleCheck) {
                rotationLockAngle = robot.GetRotation();
                coordinateSystemLock = !coordinateSystemLock;
            }
            movementModeToggleCheck = gamepad1.y;

            // drive code
            if (coordinateSystemLock) {
                telemetry.addData("Drive System", "New");

                angle = Math.toRadians(robot.GetRotation()- rotationLockAngle);
                leftDiagPower = ((-gamepad1.left_stick_y - gamepad1.left_stick_x) / sq2 * Math.sin(angle) + ((-gamepad1.left_stick_y + gamepad1.left_stick_x) / sq2) * Math.cos(angle));
                rightDiagPower = ((-(-gamepad1.left_stick_y + gamepad1.left_stick_x) / sq2) * Math.sin(angle) + ((-gamepad1.left_stick_y - gamepad1.left_stick_x) / sq2 * Math.cos(angle)));

                //robot.MoveComplex(new Double2(gamepad1.left_stick_x,gamepad1.left_stick_y),moveSpeed,gamepad1.right_stick_x,angle);
            } else {
                telemetry.addData("Drive System", "Old");

                leftDiagPower = ((-gamepad1.left_stick_y + gamepad1.left_stick_x) / sq2);
                rightDiagPower = ((-gamepad1.left_stick_y - gamepad1.left_stick_x) / sq2);

                //robot.MoveComplex(new Double2(gamepad1.left_stick_x,gamepad1.left_stick_y),moveSpeed,gamepad1.right_stick_x,0);

            }
            leftRotatePower = gamepad1.right_stick_x;
            rightRotatePower = -gamepad1.right_stick_x;
            robot.driveManager.frontLeft.setPower(moveSpeed*(leftDiagPower+leftRotatePower));
            robot.driveManager.frontRight.setPower(moveSpeed*(rightDiagPower+rightRotatePower));
            robot.driveManager.backLeft.setPower(moveSpeed*(rightDiagPower+leftRotatePower));
            robot.driveManager.backRight.setPower(moveSpeed*(leftDiagPower+rightRotatePower));

//            if (gamepad1.x != servoLastToggle) {
//                if (gamepad1.x) {
//                    fineServoControl = !fineServoControl;
//                }
//                servoLastToggle = gamepad1.x;
//            }
//
//            if (fineServoControl) {

            //rotate lunchbox up with the up dpad
            lunchboxRot -= gamepad1.dpad_up ? deltaTime.seconds() * 1 : 0;
            //rotate lunchbox down with the down dpad
            lunchboxRot += gamepad1.dpad_down ? deltaTime.seconds() * 1 : 0;
            lunchboxRot = bMath.Clamp(lunchboxRot, 0, 1);
//            robot.lunchbox.setPosition(lunchboxRot);

//            } else {
//                robot.lunchbox.setPosition(0.4333);
//                telemetry.addData("LunchboxRot Position Yeeted", lunchboxRot);
//
//            }

            /*
            if (gamepad1.a) {
                lockRotation = !lockRotation;
                sleep(100);
            }

            if (lastLockRotation != lockRotation) {
                lastLockRotation = lockRotation;
                if (lockRotation) {
                    targetRotation = robot.GetRotation();
                }
            }
*/


            //ARM CONTROLS

            //use the left bumper to make the toggle the arm controls (rectangular or polar)
//            if (gamepad2.left_bumper && !leftBumper2Check) {
//                rectControls = !rectControls;
//            }
//            leftBumper2Check = gamepad2.left_bumper;


            // Activates rectControls when right stick is being moved
            rectControls = ((Math.abs(gamepad2.right_stick_y) > 0.1) || (Math.abs(gamepad2.right_stick_x) > 0.1));
            //sets direction of rectControls to whichever axis is greater
            rectControls_goingUp = Math.abs(gamepad2.right_stick_y) > Math.abs(gamepad2.right_stick_x);

            //get new extension constants if rectControls changes or if direction changes
            if ( (rectControls != rectControlsCheck ) || (rectControls_goingUp != rectControls_goingUpCheck) )
                 robot.arm.ExtConstCalc();
            rectControlsCheck = rectControls;
            rectControls_goingUpCheck = rectControls_goingUp;


            //Allows the Gripper to be moved straight up and down with the right joystick
            if (rectControls) {
                telemetry.addLine("Arm Control: Rect");
                //set power and distance to the Arm.
                robot.arm.SetArmStatePowerCm(robot.arm.RectExtension(rectControls_goingUp),
                                           rectControls_goingUp ? gamepad2.right_stick_y : -gamepad2.right_stick_x);
            } else {
                telemetry.addLine("Arm Control: Radial");

                extension += gamepad2.right_trigger * deltaTime.seconds();    //extend arm when right trigger held
                extension -= gamepad2.left_trigger * deltaTime.seconds();     //retract arm when left trigger held

                raiseSpeed = bMath.Clamp(gamepad2.left_stick_y, -1, 1);
                robot.arm.SetArmStatePower(extension,raiseSpeed);
            }

/*
            if (rectControls) {

                //extend or shorten arm with Dpad
                if ((gamepad2.dpad_up || gamepad2.dpad_down) && !lastD2press) {
                    vertExtensionConst = robot.arm.calcVertExtensionConst();
                }
                lastD2press = gamepad2.dpad_up || gamepad2.dpad_down;

                if (gamepad2.dpad_up) {
                    vertDMove = 0.25;
                    extension = (robot.arm.calcVertExtensionTicks(vertExtensionConst) + 10) / -2613;
                }
                if (gamepad2.dpad_down) {
                    vertDMove = -0.25;
                    extension = (robot.arm.calcVertExtensionTicks(vertExtensionConst) + 10) / -2613;
                }
                if (!gamepad2.dpad_up && !gamepad2.dpad_down) {
                    vertDMove = 0;
                }
            } else {
                //extend arm by tapping right trigger
                extension += gamepad2.right_trigger * deltaTime.seconds();
                //retract arm by tapping left trigger
                extension -= gamepad2.left_trigger * deltaTime.seconds();
            }
*/

//                if (Math.abs(gamepad2.right_stick_y) > 0.1) {
//                    yWanted += deltaTime.seconds() * gamepad2.right_stick_y;
//                }
//                if (Math.abs(gamepad2.right_stick_x) > 0.1) {
//                    xWanted += deltaTime.seconds() * gamepad2.right_stick_x;
//                }
//                double armLengthNeeded = Math.sqrt(xWanted * xWanted + yWanted * yWanted);
//                double armAngleNeeded = Math.atan(yWanted / xWanted);
//                robot.arm.SetArmLengthAndAngle(armAngleNeeded, armLengthNeeded);

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
            if (gamepad1.b && !bLast) {
                if (gamepad1.b) {
                    gripFoundation = !gripFoundation;
                }
            }
            bLast = gamepad1.b;
            robot.foundationServo0.setPosition(gripFoundation ? 0 : 1);
            robot.foundationServo1.setPosition(gripFoundation ? 1 : 0);

            extension = bMath.Clamp(extension, 0, 1);
            armAngle = bMath.Clamp(armAngle, 0, 1);
            gripAngle = bMath.Clamp(gripAngle, 0, 180);





            telemetry.addLine("------ Movement ------");
            telemetry.addData("Rotation Locked ", coordinateSystemLock);
            telemetry.addData("Current Rotation ", robot.GetRotation());
            telemetry.addLine("-------- Arm  --------");
            telemetry.addData("Current Arm Angle", robot.arm.thetaAngle());
            telemetry.addData("Current Potentiometer value", robot.armPotentiometer.GetAngle());
            telemetry.addData("RectWanted?:", rectControls);
            telemetry.addLine("------ Lunchbox ------");
            telemetry.addData("Current Lunchbox", lunchboxRot);
            telemetry.update();

            deltaTime.reset();
        }
        robot.Stop();
    }
}
