package org.firstinspires.ftc.teamcode.Robot;

import android.renderscript.Double2;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Helpers.bDataManager;
import org.firstinspires.ftc.teamcode.Helpers.bMath;

import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.HashMap;
import java.util.concurrent.atomic.AtomicBoolean;

import static java.lang.Thread.sleep;

public class RobotArm extends Thread {

    LinearOpMode Op;

    //Arm height motor
    public DcMotor rotation;

    //Controls arm length (spool)
    public DcMotor length;

    public Servo gripRotation;
    public Servo grip;

    public double targetLength;
    public double currentLengthSpeed;
    public double targetLengthSpeed;


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

        grip = opMode.hardwareMap.get(Servo.class, gripServo);
        gripRotation = opMode.hardwareMap.get(Servo.class, gripRotationServo);
        rotation = opMode.hardwareMap.get(DcMotor.class, armRotationMotor);
        length = opMode.hardwareMap.get(DcMotor.class, armSpoolMotor);

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
        double H = 76.9;
        double L = 135;
        double d = (rotation.getCurrentPosition() * 0.5) / 480;
        Double c = ((k * k) - (H * H) - (L * L) - (d * d)) / 2;
        Double x = (((d * c) - (H * Math.sqrt((((L * L) * (d * d)) + ((L * L) * (H * H))) - (c * c)))) / ((d * d) + (H * H))) + d;

        return Math.atan((Math.sqrt((k * k) - (x * x)) - H) / (d - x));
    }

    public double armRectExtension(double horiz, double vert) {
        return Math.sqrt(horiz * horiz + vert * vert);
    }

    public double armRectAngle(double horiz, double vert) {
        return Math.atan(vert / horiz);
    }


    public void SetArmStateWait(double targetAngle, double _targetLength, double angleSpeed) {
        // angleSpeed really means the angle you want the arm to be
        targetLengthSpeed = 1;
        targetLength = (RobotConfiguration.arm_lengthMax * _targetLength);
        rotation.setPower(angleSpeed);

        rotation.setTargetPosition((int) (RobotConfiguration.arm_rotationMax * targetAngle));


        rotation.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        length.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        double runtime = 0;
        double rotationDelta;
        double lastrotationDelta = 1000000;
        double lengthDelta;
        double lastlengthDelta = 100000;
        ElapsedTime dt = new ElapsedTime();

        while (Op.opModeIsActive() /*&& (Math.abs(rotation.getCurrentPosition() - rotation.getTargetPosition()) > 5 || Math.abs(length.getCurrentPosition() - targetLength) > 5)*/) {

            rotation.setPower(angleSpeed);
            length.setPower(angleSpeed);
            Op.telemetry.addData("Rotation Power", rotation.getPower());
            Op.telemetry.addData("Rotation Position", rotation.getCurrentPosition());
            Op.telemetry.addData("Length Position", length.getCurrentPosition());
            Op.telemetry.addData("Rotation Goal", rotation.getTargetPosition());
            Op.telemetry.addData("Length DT", deltaTime.seconds());

            Op.telemetry.update();


            if (runtime > 0.25) {
                rotationDelta = Math.abs((int) lastrotationDelta - rotation.getCurrentPosition());
                lastrotationDelta = rotation.getCurrentPosition();

                lengthDelta = Math.abs((int) lastlengthDelta - length.getCurrentPosition());
                lastlengthDelta = length.getCurrentPosition();


                if (rotationDelta <= 3) {
                    break;
                }
                if (lengthDelta <= 3) {
                    break;
                }
            }

            runtime += dt.seconds();
            dt.reset();

        }

        rotation.setPower(0);
    }

    public void SetArmState(double targetAngle, double _targetLength, double angleSpeed) {
        // angleSpeed really means the angle you want the arm to be
        targetLengthSpeed = 1;
        targetLength = ((double) RobotConfiguration.arm_lengthMax * _targetLength);
        rotation.setPower(angleSpeed);


//        length.setTargetPosition((int) ((double) -2623 * _targetLength));

        rotation.setTargetPosition((int) ((double) RobotConfiguration.arm_rotationMax * targetAngle));
        rotation.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        length.setMode(DcMotor.RunMode.RUN_TO_POSITION);

//        currentLengthSpeed = 0;
    }

    //Set Arm Target Length and Power
    public void SetArmAnglePower(double _targetLength, double angleSpeed) {

        targetLengthSpeed = 1;
        targetLength = ((double) RobotConfiguration.arm_lengthMax * _targetLength);


        rotation.setPower(angleSpeed);
        rotation.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        length.setMode(DcMotor.RunMode.RUN_TO_POSITION);

    }

    //17.8 rotation is one CM
    //(17.8 / 480) encoder ticks is one CM
    public double calcVertExtensionConst() {
        return ((17.8 * (double) length.getCurrentPosition()) / 480 * Math.cos(thetaAngle()));
    }

    public double calcVertExtensionTicks(double k) {
        //Make sure to convert from encoder ticks when calling
        return 480 * (k / Math.cos(thetaAngle()) / 17.8);
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