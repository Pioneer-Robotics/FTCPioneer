package org.firstinspires.ftc.teamcode.Robot;

import android.renderscript.Double2;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Helpers.bMath;

import java.util.concurrent.atomic.AtomicBoolean;

import static java.lang.Thread.sleep;

public class RobotArm extends Thread {

    private LinearOpMode op;

    public Robot robot;
    //Arm height motor
    public DcMotor rotation;

    //useful for knowing the position of the arm
    double k = 177.0;
    double h = 32.2; //Vertical Distance from bottom joint of arm to the axis made by the center of the lead screw
    double l = 134.0; //Distance from bottom joint of arm to middle joint on Xrail

    double pot;

    //Controls arm length (spool)
    public DcMotor length;

    public Servo gripRotation;
    public Servo grip;

    public ArmThreadMode rotationMode = ArmThreadMode.Enabled;
    public ArmThreadMode extensionMode = ArmThreadMode.Enabled;

    //Arm rotation mode, setting to 'threaded' will arm rotation handled by a thread while 'disabled' will not effect arm rotation
    public enum ArmThreadMode {
        Enabled,
        Disabled
    }


    public double targetLength;
    public double targetRotation;
    public double targetLengthSpeed;

    private boolean protectSpool = true;


    public enum GripState {
        OPEN,
        IDLE,
        CLOSED
    }

    // lets make some real changes
    private ElapsedTime deltaTime = new ElapsedTime();

    //The scale range Double2's are interpreted as X = min and Y = max.
    public RobotArm(LinearOpMode opMode, String armRotationMotor, String armSpoolMotor, String gripServo, String gripRotationServo, Double2 gripRange, Double2 gripRotationRange) {
        op = opMode;
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
    //I think "usePot" math is off by 90 radians, am subtracting 90 radians

    public double thetaAngle() {
        double potentiometerMeasurement = bMath.toRadians(robot.armPotentiometer.getAngle());

        potentiometerMeasurement = bMath.Clamp(potentiometerMeasurement, 0, 3.141);
        double C0 = bMath.squared(l) + bMath.squared(k) - (2 * k * l * Math.cos(potentiometerMeasurement));
        double C = Math.sqrt(C0);
        double Numerator1 = bMath.squared(l) + bMath.squared(C) - bMath.squared(k);
        double thetaPart1 = Math.acos(Numerator1 / 2 / C / l);
        double thetaPart2 = Math.acos(h / C);
        return thetaPart1 + thetaPart2 - (Math.PI / 2);

    }

    /*
    "realPotentiometerAngle" takes as an input the length of d measured in cm
    it outputs the angle the potentiometer should be measuring in radians
    this function can be useful for checking the accuracy of the potentiometer
     */
    public double derivedPotentiometerAngle(double dLengthIn_cm) {
        double d = dLengthIn_cm;
        double intermedieteVal1 = (l * l) + (k * k) - (d * d) - (h * h);
        double intermedVal2 = intermedieteVal1 / 2 / k / l;
        return Math.acos(intermedVal2);
    }

    /*
    This function returns the horizotntal distance between the shuttle joint and the arm joint.
    can be plugged into realPotentiometerAngle to determine what the pot should be reading.
     */
    public double currentArmQuadBaseDistance() {
        return RobotConfiguration.armQuadBaseMaxCm - ((double) rotation.getCurrentPosition() * (1.0 / 1440.0) * (120.0 / 48.0) * (2 / 1.0));
    }


    /*
    This function drives the arm up or down to a desired angle.
    put that angle between 0 and PI/2 (in radians)
    not exact, we try to get it within a certain threshold but the arm jerks
     */
    @Deprecated
    private void runToTheta(double thetaWanted) //FYI the way this is written, trying to change thetaAngle smoothly will cause it to jump in steps
    {
        double thetaThreshold = Math.PI * (2.0 / 180.0);
        double thetaPower = 0.25;
        rotation.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //depending on if the angle needs to be increased or decreased, turn on the motors
        if (Math.abs(thetaAngle() - thetaWanted) > thetaThreshold) { //if the rotation is not yet at the intended position
            if (thetaAngle() > thetaWanted)
                rotation.setPower(thetaPower);
            else
                rotation.setPower(-thetaPower);
        } else rotation.setPower(0);
            /*
            make sure the current angle doesn't exceed max/min
             */
    }


    /*
    This method will move the arm to match a desired length and angle.
    angle should be between 0 and PI/2 (measured in radians)
    length should be specified in cm. Should be between 0 and 100.
     */
    @Deprecated
    public void SetArmLengthAndAngle(double angleNeeded, double lengthNeeded) {
        //cmToRange is what you use to convert from cm to the 0-1 scale we use to actually set the arm length
            /*
            This is where we put the stuff to convert our cm value to weird Ben value
             */
        length.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        length.setTargetPosition((int) lengthNeeded);
        runToTheta(angleNeeded);
    }


    public void setArmStateWait(double targetAngle, double _targetLength) {
        targetLengthSpeed = 1;
        targetLength = (RobotConfiguration.arm_ticksMax * _targetLength);
        targetRotation = (RobotConfiguration.arm_rotationMax * targetAngle);

        rotation.setTargetPosition((int) (RobotConfiguration.arm_rotationMax * targetAngle));


        rotation.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        length.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        while (op.opModeIsActive() && (!armRotationTargetReached() || !armLengthTargetReached())) {

        }

        rotation.setPower(0);
    }


    @Deprecated
    public void setArmStateWaitCm(double targetAngle, double _targetLength) {
        targetLengthSpeed = 1;
        targetLength = cmToTicks(_targetLength);


        targetRotation = (RobotConfiguration.arm_rotationMax * targetAngle);

        rotation.setTargetPosition((int) (RobotConfiguration.arm_rotationMax * targetAngle));


        rotation.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        length.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        while (op.opModeIsActive() && (!armRotationTargetReached() || !armLengthTargetReached())) {

        }

        rotation.setPower(0);
    }

    public void setArmStateAsync(double targetAngle, double _targetLength) {
        targetLengthSpeed = 1;
        targetLength = (RobotConfiguration.arm_ticksMax * _targetLength);
        targetRotation = (RobotConfiguration.arm_rotationMax * targetAngle);

        rotation.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        length.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void setArmStateAsyncCm(double targetAngle, double _targetLength) {
        targetLengthSpeed = 1;
        targetLength = cmToTicks(_targetLength);
        targetRotation = (RobotConfiguration.arm_rotationMax * targetAngle);

        rotation.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        length.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }


    public boolean armRotationTargetReached() {
        return Math.abs(rotation.getCurrentPosition() - targetRotation) < 10;
    }


    public boolean armLengthTargetReached() {
        return Math.abs(length.getCurrentPosition() - length.getTargetPosition()) < 10;
    }

    public boolean armAsyncTargetReached() {
        return armRotationTargetReached() && armLengthTargetReached();
    }

    public void setSpoolProtect(boolean _spoolProtect) {
        protectSpool = _spoolProtect;
    }

    /*
    This method moves the arm to an extension represented in % fully extended from 0 to 1
    and moves the shuttle on the lead screw to a position represented in % fully up from 0 to 1
     */

    public void SetArmState(double targetAngle, double _targetLength, double angleSpeed) {
        // angleSpeed really means the angle you want the arm to be
        targetLengthSpeed = 1;
        targetLength = (RobotConfiguration.arm_ticksMax * _targetLength);

        if (targetLength < 0 && protectSpool)
            targetLength = 0; //don't extend the spool past it's starting point

        rotation.setPower(angleSpeed);
        rotation.setTargetPosition((int) ((double) RobotConfiguration.arm_rotationMax * targetAngle));
        rotation.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        length.setTargetPosition((int) targetLength);
        length.setMode(DcMotor.RunMode.RUN_TO_POSITION);

    }


    /*
    This method will rotate the arm at a specified speed and extend
    to match a certain percentage of extension between 0 and 1
     */
    public void SetArmStatePower(double _targetLength, double angleSpeed) {
        targetLengthSpeed = 1;
        targetLength = (RobotConfiguration.arm_ticksMax * _targetLength);
        if (targetLength < 0 && protectSpool)
            targetLength = 0; //don't extend the spool past it's starting point

        rotation.setPower(angleSpeed);
        rotation.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        length.setPower(targetLengthSpeed);
        length.setTargetPosition((int) targetLength);
        length.setMode(DcMotor.RunMode.RUN_TO_POSITION);

    }

    public void SetArmStateExtensionPower(double lengthSpeed, double angleSpeed) {

        rotation.setPower(angleSpeed);
        rotation.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        if (length.getCurrentPosition() / RobotConfiguration.arm_ticksMax < 0 && protectSpool) {
            lengthSpeed = bMath.Clamp(lengthSpeed, 0, 1 );
        } else if (length.getCurrentPosition() / RobotConfiguration.arm_ticksMax > 1 && protectSpool){
            lengthSpeed = bMath.Clamp(lengthSpeed, -1, 1 );
        }

        robot.arm.length.setPower(lengthSpeed);
        length.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        length.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }

    /*
    This method will rotate the arm at a specified speed and extend
    to match a certain distance in cm >0
     */
    public void SetArmStatePowerCm(double _targetLength, double angleSpeed) {
        targetLengthSpeed = 1; //speed of extension
        targetLength = cmToTicks(_targetLength);
        if (targetLength > 0 && protectSpool)
            targetLength = 0; //don't extend the spool past it's starting point


        rotation.setPower(angleSpeed);
        rotation.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        length.setPower(1);
        length.setTargetPosition((int) targetLength);
        length.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    /*
    The following two methods convert ticks to cm and vice versa

    17.8 cm is one rotation
     n cm * (1rot/17.8cm) * (480tick/1rot) = n * (480/17.8)(tick/cm)
    (17.8 / 480) cm is one tick, (480/17.8)tick is one cm
    */

    public double ticksToCm(int ticks) {
        return (double) ticks * (15.7 / 960) + RobotConfiguration.arm_lengthMin;
    }

    public int cmToTicks(double cm) {
        return (int) ((cm - RobotConfiguration.arm_lengthMin) * (960 / 15.7));
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
    public double xExtConst() {
        return ticksToCm(length.getCurrentPosition()) * Math.cos(thetaAngle());
    }

    public double yExtConst() {
        return ticksToCm(length.getCurrentPosition()) * Math.sin(thetaAngle());
    }


    //returns the amount the arm should be extended when moving (in cm)
    public double RectExtension(boolean goingUp, double xExtConst, double yExtConst) {
        if (goingUp)
            return xExtConst / bMath.Clamp(Math.cos(thetaAngle()), 0.0001, 1);
        else
            return yExtConst / bMath.Clamp(Math.sin(thetaAngle()), 0.0001, 1);
    }


    public void setGripState(GripState gripState, double rotationPosition) {
        grip.setPosition(gripState == GripState.CLOSED ? 0 : (gripState == GripState.IDLE ? 0.4 : 0.74));
        gripRotation.setPosition(rotationPosition);
    }

}
//oof
