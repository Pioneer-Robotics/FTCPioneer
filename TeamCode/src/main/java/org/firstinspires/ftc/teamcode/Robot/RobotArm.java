package org.firstinspires.ftc.teamcode.Robot;

import android.renderscript.Double2;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.concurrent.atomic.AtomicBoolean;

import static java.lang.Thread.sleep;

public class RobotArm extends Thread {

    LinearOpMode Op;

    public Robot robot;
    //Arm height motor
    public DcMotor rotation;

    //Controls arm length (spool)
    public DcMotor length;

    public Servo gripRotation;
    public Servo grip;

    public double targetLength;
    public double targetLengthSpeed;
    public double xExtConst;
    public double yExtConst;
    public double pot;

    public boolean protectSpool = true;

    public boolean usePot = true;

    public enum GripState {
        OPEN,
        IDLE,
        CLOSED
    }

    // lets make some real changes
    AtomicBoolean runningThread = new AtomicBoolean();

    ElapsedTime deltaTime = new ElapsedTime();

    //The scale range Double2's are interpreted as X = min and Y = max.
    public RobotArm(LinearOpMode opMode, String armRotationMotor, String armSpoolMotor, String gripServo, String gripRotationServo, Double2 gripRange, Double2 gripRotationRange) {
        Op = opMode;
        robot = Robot.instance;

        grip = opMode.hardwareMap.get(Servo.class, gripServo);
        gripRotation = opMode.hardwareMap.get(Servo.class, gripRotationServo);
        rotation = opMode.hardwareMap.get(DcMotor.class, armRotationMotor);
        length = opMode.hardwareMap.get(DcMotor.class, armSpoolMotor);
        pot = robot.armPotentiometer.getAngle();

        grip.scaleRange(gripRange.x, gripRange.y);
        gripRotation.scaleRange(gripRotationRange.x, gripRotationRange.y);


        rotation.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        length.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        rotation.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        length.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        length.setTargetPosition(0);
        rotation.setTargetPosition(0);

        rotation.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        length.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    //Returns the angle that the arm is at. Please verify this math typing.
    public double thetaAngle() {
        double k = 177;
        double h = 76.9;
        double l = 135;
        double C = pot + RobotConfiguration.pot_interiorOffset;

        if (usePot) {
            Double c = Math.sqrt((k * k) + (l * l) - 2 * k * l * Math.cos(C));
            return Math.asin((k * Math.sin(C)) / c) + 90 - Math.asin(h / c);
        } else {
            double d = (rotation.getCurrentPosition() * 0.5) / 480; //TODO add offset to this value so it actually works lol: starts at 0 rn
            Double c = ((k * k) - (h * h) - (l * l) - (d * d)) / 2;
            Double x = (((d * c) - (h * Math.sqrt((((l * l) * (d * d)) + ((l * l) * (h * h))) - (c * c)))) / ((d * d) + (h * h))) + d;

            return Math.atan((Math.sqrt((k * k) - (x * x)) - h) / (d - x));
        }

    }

    /*
    This function drives the arm up or down to a desired angle.
    put that angle between 0 and PI/2 (in radians)
    not exact, we try to get it within a certain threshold but the arm jerks
     */
    public void runToTheta(double thetaWanted) //FYI the way this is written, trying to change thetaAngle smoothly will cause it to jump in steps
    {
        double thetaThreshold = Math.PI * (5 / 180);
        double thetaPower = 0.25;
        rotation.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //depending on if the angle needs to be increased or decreased, turn on the motors
        if (Math.abs(thetaAngle() - thetaWanted) > thetaThreshold) { //if the rotation is not yet at the intended position
            if (thetaAngle() - thetaWanted > 0)
                rotation.setPower(thetaPower);
            else
                rotation.setPower(-thetaPower);
        }
        rotation.setPower(0);
    }

    /*
    This method will move the arm to match a desired length and angle.
    angle should be between 0 and PI/2 (measured in radians)
    length should be specified in cm. Should be between 0 and 100.
     */
    public void SetArmLengthAndAngle(double angleNeeded, double lengthNeeded) {
        //cmToRange is what you use to convert from cm to the 0-1 scale we use to actually set the arm length
            /*
            This is where we put the stuff to convert our cm value to weird Ben value
             */
        length.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        length.setTargetPosition((int) lengthNeeded);
        runToTheta(angleNeeded);
    }


    public void SetArmStateWait(double targetAngle, double _targetLength, double angleSpeed) {
        // angleSpeed really means the angle you want the arm to be
        targetLengthSpeed = 1;
        targetLength = (RobotConfiguration.arm_ticksMax * _targetLength);
        rotation.setPower(angleSpeed);

        rotation.setTargetPosition((int) (RobotConfiguration.arm_rotationMax * targetAngle));


        rotation.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        length.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        double runtime = 0;
        double rotationDelta = 0;
        double lastrotationDelta = 1000000;
        double lengthDelta = 0;
        double lastlengthDelta = 100000;
        ElapsedTime dt = new ElapsedTime();

        dt.reset();

        while (Op.opModeIsActive() /*&& (Math.abs(rotation.getCurrentPosition() - rotation.getTargetPosition()) > 5 || Math.abs(length.getCurrentPosition() - targetLength) > 5)*/) {

            rotation.setPower(angleSpeed);
            length.setPower(angleSpeed);
            Op.telemetry.addData("Rotation Power", rotation.getPower());
            Op.telemetry.addData("Rotation Position", rotation.getCurrentPosition());
            Op.telemetry.addData("Length Position", length.getCurrentPosition());
            Op.telemetry.addData("Rotation Goal", rotation.getTargetPosition());
            Op.telemetry.addData("Rotation Delta", rotationDelta);
            Op.telemetry.addData("Length Delta", lengthDelta);


            Op.telemetry.addData("Length DT", deltaTime.seconds());

            Op.telemetry.update();


            if (runtime > 0.25) {

                Op.telemetry.addData("Arm Telem", rotationDelta);

                rotationDelta = Math.abs((int) lastrotationDelta - rotation.getCurrentPosition());
                lastrotationDelta = rotation.getCurrentPosition();

                lengthDelta = Math.abs((int) lastlengthDelta - length.getCurrentPosition());
                lastlengthDelta = length.getCurrentPosition();

                if (rotationDelta <= 3 && lengthDelta <= 3) {
                    break;
                }
            }

            runtime += dt.seconds();
            dt.reset();

        }

        rotation.setPower(0);
    }

    /*
    This method moves the arm to an extension represented in % fully extended from 0 to 1
    and moves the shuttle on the lead screw to a position represented in % fully up from 0 to 1
     */
    public void SetArmState(double targetAngle, double _targetLength, double angleSpeed) {
        // angleSpeed really means the angle you want the arm to be
        targetLengthSpeed = 1;
        targetLength = (RobotConfiguration.arm_ticksMax * _targetLength);
        if (targetLength > 0 && protectSpool)
            targetLength = 0; //don't extend the spool past it's starting point

        rotation.setPower(angleSpeed);
        rotation.setTargetPosition((int) ((double) RobotConfiguration.arm_rotationMax * targetAngle));
        rotation.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        length.setMode(DcMotor.RunMode.RUN_TO_POSITION);

    }


    /*
    This method will rotate the arm at a specified speed and extend
    to match a certain percentage of extension between 0 and 1
     */
    public void SetArmStatePower(double _targetLength, double angleSpeed) {

        targetLengthSpeed = 1;
        targetLength = (RobotConfiguration.arm_ticksMax * _targetLength);
        if (targetLength > 0 && protectSpool)
            targetLength = 0; //don't extend the spool past it's starting point

        rotation.setPower(angleSpeed);
        rotation.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        length.setMode(DcMotor.RunMode.RUN_TO_POSITION);

    }

    /*
    This method will rotate the arm at a specified speed and extend
    to match a certain distance in cm >0
     */
    public void SetArmStatePowerCm(double _targetLength, double angleSpeed) {
        targetLengthSpeed = 1; //speed of extension
        targetLength = CmToTicks(_targetLength);
        if (targetLength > 0 && protectSpool)
            targetLength = 0; //don't extend the spool past it's starting point


        rotation.setPower(angleSpeed);
        rotation.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        length.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    /*
    The following two methods convert ticks to cm and vice versa

    17.8 cm is one rotation
     n cm * (1rot/17.8cm) * (480tick/1rot) = n * (480/17.8)(tick/cm)
    (17.8 / 480) cm is one tick, (480/17.8)tick is one cm
    */

    public double TicksToCm(int ticks) {
        return (double) ticks * (17.8 / 480) + RobotConfiguration.arm_lengthMin;
    }

    public int CmToTicks(double cm) {
        return (int) (cm * (480 / 17.8) - RobotConfiguration.arm_lengthMin);
    }

/* Principle for rectangular control
         /|
     l /  |
     /    | y
   / A    |
   --------
      x
      cos(A)= x/l   ;   l = x/cos(A)   ;   x = l*cos(A)    --Keep x constant, change A, get vertical movement
      sin(A)= y/l   ;   l = y/sin(A)   ;   y = l*sin(A)    --Keep y constant, change A, get horizontal movement
 */


    //returns and sets the above x and y values (in cm)
    public void ExtConstCalc() {
        xExtConst = TicksToCm(length.getCurrentPosition()) * Math.cos(thetaAngle());

        yExtConst = TicksToCm(length.getCurrentPosition()) * Math.sin(thetaAngle());
    }


    //returns the amount the arm should be extended when moving (in cm)
    public double RectExtension(boolean goingUp) {
        if (goingUp)
            return xExtConst / Math.cos(thetaAngle());
        else
            return yExtConst / Math.sin(thetaAngle());
    }


    public void SetGripState(GripState gripState, double rotationPosition) {
        grip.setPosition(gripState == GripState.CLOSED ? 0 : (gripState == GripState.IDLE ? 0.23 : 0.64));
        gripRotation.setPosition(rotationPosition);
    }

    public void Stop() {
        runningThread.set(false);
    }
}
//oof