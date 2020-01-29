package org.firstinspires.ftc.teamcode.Robot;

import android.renderscript.Double2;
import android.renderscript.Double4;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Helpers.PID;
import org.firstinspires.ftc.teamcode.Helpers.bDataManager;
import org.firstinspires.ftc.teamcode.Helpers.bMath;
import org.firstinspires.ftc.teamcode.Helpers.bTelemetry;
import org.firstinspires.ftc.teamcode.Hardware.bIMU;
import org.firstinspires.ftc.teamcode.Hardware.Potentiometer;
import org.firstinspires.ftc.teamcode.Robot.Input.RobotInputThread;

import java.util.concurrent.atomic.AtomicBoolean;

//TODO: clean up the canmove system
public class Robot extends Thread {

    //Static instance. Only have one robot at a time and access it from here (THERE CAN BE ONLY ONE)
    public static Robot instance;

    //The linear slide arm, controls length and angle
    public RobotArm arm;

    //WIP WIP WIP, should let us have really responsive sensor data (right now IMU and distance)
    public RobotInputThread experimentalInput = new RobotInputThread();

    //The wall tracker, lets you track along a wall using a sensor group and other data
    public RobotWallTrack wallTrack = new RobotWallTrack();

    public Potentiometer armPotentiometer = null;

    public Servo capstoneServo;
    public Servo foundationServo0;
    public Servo foundationServo1;

    //The current IMU rotation, assigned by a thread
    static double rotation;

    //The IMU reader, takes the average of 2 IMU's to give a fancy (and likely less preferment) reading!
    public bIMU imu = new bIMU();

    //The wheel drive manager, makes sure all wheels are moving at the same speed at all times
    public RobotDriveManager driveManager;

    //Delta time, used in the thread for timing
    public ElapsedTime threadDeltaTime = new ElapsedTime();


    //The data manager serves to store data locally on the phone, used in calibration and PID tuning.
    private bDataManager dataManger = new bDataManager();

    double desiredArmRotationPower;

    public LinearOpMode Op;
//    public OpMode LinearOpMode;

    //If our thread is running, using atomics to avoid thread conflicts. Might not be completely necessary
    private AtomicBoolean threadRunning = new AtomicBoolean();

    private PID rotationPID = new PID();

    public void init(LinearOpMode opmode, boolean useWalltrack) {
        //start the printer service
        bTelemetry.start(opmode);

        //Fail safe to make sure there is only one Robot.java running.
//        if (instance != null) {
//            bTelemetry.print("FATAL ERROR: THERE CAN ONLY BE ONE INSTANCE OF ROBOT.JAVA");
//            return;
//        }


        //Set up the instance
        instance = this;
        bTelemetry.print("Robot instance assigned.");

        //Set the opmode
        Op = opmode;

        getHardware(opmode, useWalltrack);

        capstoneServo.setPosition(0.733);

        //Starts the 'run' thread
        start();
        bTelemetry.print("Robot thread initialized.");

        bTelemetry.print("Robot start up successful. Preparing to read wheel calibration data...");

        //Starts the dataManager to read calibration data
        dataManger.Start();

        bTelemetry.print("bDataManager started.");


        //Assign and display calibration data for debugging purposes
        driveManager.frontLeft.powerCoefficent = dataManger.readData("wheel_front_left_powerCo", -1);
        bTelemetry.print("      Front Left  : " + driveManager.frontLeft.powerCoefficent);
        driveManager.frontRight.powerCoefficent = dataManger.readData("wheel_front_right_powerCo", -1);
        bTelemetry.print("      Front Right : " + driveManager.frontRight.powerCoefficent);
        driveManager.backLeft.powerCoefficent = dataManger.readData("wheel_back_left_powerCo", -1);
        bTelemetry.print("      Back Left   : " + driveManager.backLeft.powerCoefficent);
        driveManager.backRight.powerCoefficent = dataManger.readData("wheel_back_right_powerCo", -1);
        bTelemetry.print("      Back Right  : " + driveManager.backRight.powerCoefficent);


        //Adds the motors and distance sensors to the expInput manager to allow for faster reads
        //DISABLED BUT WORKS
//        bTelemetry.print("Initializing Experimental Input...");
//        for (bMotor motor : driveManager.driveMotors) {
//            experimentalInput.AddMotor(motor);
//        }
//
//        for (DistanceSensor sensor : wallTrack.sensors) {
//            experimentalInput.AddSensor(sensor);
//        }

        arm.SetGripState(RobotArm.GripState.IDLE, 0.8);
        setFoundationGripperState(0);

        bTelemetry.print("Wheel boot successful. Ready to operate!");
    }

    public void getHardware(LinearOpMode opmode, boolean useWallTracking) {

        //Sets up the drive train hardware
        bTelemetry.print("Configuring drive train...");
        driveManager = new RobotDriveManager(opmode, RobotConfiguration.wheel_frontLeft, RobotConfiguration.wheel_frontRight, RobotConfiguration.wheel_backLeft, RobotConfiguration.wheel_backRight);

        //Invert the left side wheels
        driveManager.frontLeft.setDirection(DcMotor.Direction.REVERSE);
        driveManager.backLeft.setDirection(DcMotor.Direction.REVERSE);

        //Reset drive train encoders
        setDriveMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setDriveMode(DcMotor.RunMode.RUN_USING_ENCODER);


        //Sets up the arms hardware
        bTelemetry.print("Configuring arm motors...");
        armPotentiometer = new Potentiometer(opmode, RobotConfiguration.armPotentiometer);
        arm = new RobotArm(opmode, RobotConfiguration.arm_rotationMotor, RobotConfiguration.arm_lengthMotor, RobotConfiguration.arm_gripServo, RobotConfiguration.arm_gripRotationServo, new Double2(0, 1), new Double2(0, 1));

        capstoneServo = opmode.hardwareMap.get(Servo.class, RobotConfiguration.capstoneServo);

        bTelemetry.print("Configuring IMU...");
        imu.Start(opmode);

        if (useWallTracking) {
            bTelemetry.print("Configuring wall tracking...");
            wallTrack.Start(opmode);
        }
        foundationServo0 = opmode.hardwareMap.get(ServoImplEx.class, RobotConfiguration.foundationGrip0);
        foundationServo1 = opmode.hardwareMap.get(Servo.class, RobotConfiguration.foundationGrip1);

        setFoundationGripperState(1);


        while (imu.initStatus.get() < 2) {

        }

        bTelemetry.print("Completed IMU start up.");

        while (useWallTracking && !wallTrack.startUpComplete.get()) {

        }

        bTelemetry.print("Completed walltrack start up.");

        bTelemetry.print("Hardware configuration complete.");
    }


    //A fancy version of init used for calibrating the robot, not to be used in any offical match as calibration will take anywhere from 10 to 30 seconds
    public void initCalibration(HardwareMap hardwareMap, LinearOpMode opmode) {

        //start the printer
        bTelemetry.start(opmode);

        //Set up the instance (safety checks might be a good idea at some point)
        instance = this;
        bTelemetry.print("Robot instance assigned.");

        //Set the opmode
        Op = opmode;

        getHardware(opmode, false);
        setFoundationGripperState(0);

//        //Find the motors
//        driveManager = new RobotDriveManager(opmode, RobotConfiguration.wheel_frontLeft, RobotConfiguration.wheel_frontRight, RobotConfiguration.wheel_backLeft, RobotConfiguration.wheel_backRight);
//
//        bTelemetry.print("Robot wheels assigned.");
//        bTelemetry.print("Robot motors configured in the DriveManager.");
//
//        //Left wheels are reversed so power 1,1,1,1 moves us forward
//        driveManager.frontLeft.setDirection(DcMotor.Direction.REVERSE);
//        driveManager.backLeft.setDirection(DcMotor.Direction.REVERSE);
//
//        //Define the arm values for motors and servos (also includes ranges)
//        arm = new RobotArm(opmode, RobotConfiguration.arm_rotationMotor, RobotConfiguration.arm_lengthMotor, RobotConfiguration.arm_gripServo, RobotConfiguration.arm_gripRotationServo, new Double2(0, 1), new Double2(0, 1));
//
//        //start the thread that is responsible for fighting gravity and keeping arm position level.
////        arm.start();
//
//        //Init the motors for use.
//        setDriveMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        setDriveMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        bTelemetry.print("Wheel encoders initialized.");
//
//
//        //Set up the IMU(s)
//        imu.start(opmode);
//        bTelemetry.print("IMU's initialized.");
//
//        //Set up the wall tracker, this uses ALL the lasers so make sure they all work before running this
//        wallTrack.start(opmode);
//        bTelemetry.print("Walltracker initialized.");
//
        //Starts the 'run' thread
        start();
        bTelemetry.print("Robot thread initialized.");

        bTelemetry.print("Robot start up successful. Preparing for initial wheel calibration!");

        dataManger.Start();

        bTelemetry.print("bDataManager started.");

        bTelemetry.print("Robot start up successful. Running initial wheel calibration...");

        setFoundationGripperState(0);

        driveManager.PerformInitialCalibration();

        bTelemetry.print("Wheel boot successful. Writing results...");

        dataManger.writeData("wheel_front_left_powerCo", driveManager.frontLeft.powerCoefficent);
        dataManger.writeData("wheel_front_right_powerCo", driveManager.frontRight.powerCoefficent);
        dataManger.writeData("wheel_back_left_powerCo", driveManager.backLeft.powerCoefficent);
        dataManger.writeData("wheel_back_right_powerCo", driveManager.backRight.powerCoefficent);

        bTelemetry.print("Wheel write successful.");

        bTelemetry.print("Calibration complete, pleasure doing business with you.");


    }


    //Threaded run method, right now this is just for IMU stuff, at some point we might put some avoidance stuff in here (background wall tracking?)
    public void run() {
        threadRunning.set(true);

        while (threadRunning.get()) {

            //Update our 'rotation' value
            updateBackgroundRotation();

//            threadTimer += threadDeltaTime.seconds();
//            op.telemetry.update();

            //Make sure that the robot stops once we request a stop
            if (Op.isStopRequested()) {
                setPowerDouble4(0, 0, 0, 0, 0);
                threadRunning.set(false);
            }

            arm.length.setPower(1);
            arm.length.setTargetPosition((int) arm.targetLength);

            if (arm.rotationMode == RobotArm.ArmRotationMode.Threaded) {
                desiredArmRotationPower = ((arm.rotation.getCurrentPosition() - arm.targetRotation) / (RobotConfiguration.arm_rotationMax * 0.5)) * 1;
                arm.rotation.setPower(bMath.Clamp(desiredArmRotationPower, Math.copySign(0.1, desiredArmRotationPower), Math.copySign(1, desiredArmRotationPower)));
            }
        }


    }


    public void updateBackgroundRotation() {
        //Updates the current rotation
        rotation = imu.getRotation(AngleUnit.DEGREES);
    }


    public void shutdown() {
        arm.Stop();
        experimentalInput.Stop();
        threadRunning.set(false);
        setPowerDouble4(0, 0, 0, 0, 0);
        setDriveMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }


    //<editor-fold desc="Movement">

    /**
     * Uses
     *
     * @param headingAngle  The angle  that we want to move along, try to keep its magnitude under 180
     * @param movementSpeed How fast we want to move to move along 'headingAngle'. 1 is very fast, 0 is anti-fast (brakes).
     */

    public void moveSimple(double headingAngle, double movementSpeed) {
        Double4 v = bMath.getMecMovementSimple(headingAngle);
        setPowerDouble4(v, movementSpeed);
    }

    /**
     * Uses
     *
     * @param headingVector The vector that we want to move along
     * @param movementSpeed How fast we want to move to move along 'movementAngle'. 1 is very fast, 0 is anti-fast (brakes).
     */

    public void moveSimple(Double2 headingVector, double movementSpeed) {
        Double4 v = bMath.getMecMovementSimple(headingVector);
        setPowerDouble4(v, movementSpeed);
    }

    /**
     * Uses
     *
     * @param headingAngle  The angle  that we want to move along, try to keep its magnitude under 180
     * @param movementSpeed How fast we want to move to move along 'headingAngle'. 1 is very fast, 0 is anti-fast (brakes).
     */

    public void moveSimple(double headingAngle, double movementSpeed, double rotationPower) {
        Double4 v = bMath.getMecMovementSimple(headingAngle, rotationPower);
        setPowerDouble4(v, movementSpeed);
    }

    public void moveSimple(Double2 headingVector, double movementSpeed, double rotationSpeed) {
        Double4 v = bMath.getMecMovementSimple(headingVector, rotationSpeed);
        setPowerDouble4(v, movementSpeed);
    }


    public void moveComplex(double headingAngle, double movementSpeed, double rotationSpeed, double offsetAngle) {
        Double4 v = bMath.getMecMovement(headingAngle, rotationSpeed, offsetAngle);
        setPowerDouble4(v, movementSpeed);
    }


    public void moveComplex(Double2 headingVector, double movementSpeed, double rotationSpeed, double offsetAngle) {
        Double4 v = bMath.getMecMovement(headingVector, rotationSpeed, offsetAngle);
        setPowerDouble4(v, movementSpeed);
    }

    public void rotateSimple(double rotationSpeed) {
        Double4 v = bMath.getRotationSimple(rotationSpeed);
        setPowerDouble4(v, 1);
    }


    //
    public void rotatePID(double targetAngle, double rotationSpeed, double maxTime) {

        //P of 3 and 0 for other gains seems to work really well
//        rotationPID.start(3, 0, 0.1);

        rotationPID.start(4.02, 0.0032, 0.0876);
//        rotationPID.start(4.01, 0.003, 0.0876);

//        rotationPID.start(1, 0.075, 0.022);

//        rotationPID.start(3, 0.21, 0.69);
//        rotationPID.start(0.5, 0.075, 0.015);
//        rotationPID.start(1, 0.25, 0.035);
//        rotationPID.start(0.025, 0.005, 0);

        double ticker = 0;
        double startAngle = rotation;
        int directionChanges = 0;
        boolean lastPositiveState = true;
        double rotationPower = 0;
        ElapsedTime dt = new ElapsedTime();

        double correctTime = 0;

        while (ticker < maxTime && Op.opModeIsActive()) {
            rotationPower = rotationPID.loop(targetAngle, rotation);
            rotationPower = rotationPower / (360);//rotationSpeed * Math.abs(startAngle - targetAngle));
            rotationPower += (0.03 * (rotationPower > 0 ? 1 : -1));
            Op.telemetry.addData("Error ", rotationPID.error);
            Op.telemetry.addData("Last Error  ", rotationPID.lastError);
            Op.telemetry.addData("Derivative ", rotationPID.derivative);
            Op.telemetry.addData("Integral ", rotationPID.integral);

            Op.telemetry.addData("TD ", rotationPID.deltaTime.seconds());

            Op.telemetry.addData("Rotation ", rotation);
            Op.telemetry.addData("rotationPower ", rotationPower);
            Op.telemetry.addData("rotationSpeed ", rotationSpeed);
            Op.telemetry.addData("yeets", directionChanges);
            Op.telemetry.update();
            rotateSimple(rotationPower * rotationSpeed);

            if (lastPositiveState != rotationPower > 0) {
                directionChanges++;
                lastPositiveState = rotationPower > 0;
            }

            if (Math.abs(getRotation() - targetAngle) < 0.15 * rotationPower) {
                break;
            }

            if (directionChanges > 3) {
                break;
            }

//            if (rotationPID.error < 2) {
//                correctTime += dt.seconds();
//            }
//
//            if (correctTime > 0.25) {
//                break;
//            }
//
//            if (directionChanges > 3) {
//                ticker += cycles * 2;
//                op.telemetry.addData("Rotation ended", directionChanges);
//                op.telemetry.update();
//            }
            ticker += dt.seconds();
            dt.reset();
        }

        setPowerDouble4(0, 0, 0, 0, 0);
    }

    public void rotatePIDRelative(double relativeTargetAngle, double rotationSpeed, double maxTime) {

        //P of 3 and 0 for other gains seems to work really well
//        rotationPID.start(3, 0, 0.1);

        rotationPID.start(4.02, 0.0032, 0.0876);
//        rotationPID.start(4.01, 0.003, 0.0876);

//        rotationPID.start(1, 0.075, 0.022);

//        rotationPID.start(3, 0.21, 0.69);
//        rotationPID.start(0.5, 0.075, 0.015);
//        rotationPID.start(1, 0.25, 0.035);
//        rotationPID.start(0.025, 0.005, 0);

        double ticker = 0;
        double startAngle = getRotation();
        double targetAngle = ((startAngle + relativeTargetAngle + 360) % 360) - 360;

        targetAngle = bMath.Loop(targetAngle, 180);

        int directionChanges = 0;
        boolean lastPositiveState = true;
        double rotationPower = 0;
        ElapsedTime dt = new ElapsedTime();


        while (ticker < maxTime && Op.opModeIsActive()) {
            rotationPower = rotationPID.loop(targetAngle, rotation);
            rotationPower = rotationPower / (360);//rotationSpeed * Math.abs(startAngle - relativeTargetAngle));
            rotationPower += (Math.copySign(0.1, rotationPower));
            Op.telemetry.addData("Error ", rotationPID.error);
            Op.telemetry.addData("Last Error  ", rotationPID.lastError);
            Op.telemetry.addData("Derivative ", rotationPID.derivative);
            Op.telemetry.addData("Integral ", rotationPID.integral);

            Op.telemetry.addData("TD ", rotationPID.deltaTime.seconds());

            Op.telemetry.addData("Rotation ", rotation);
            Op.telemetry.addData("rotationPower ", rotationPower);
            Op.telemetry.addData("rotationSpeed ", rotationSpeed);
            Op.telemetry.addData("yeets", directionChanges);
            Op.telemetry.update();
            rotateSimple(rotationPower * rotationSpeed);

            if (lastPositiveState != rotationPower > 0) {
                directionChanges++;
                lastPositiveState = rotationPower > 0;
            }

            if (Math.abs(getRotation() - targetAngle) < 0.15 * rotationPower) {
                break;
            }

            if (directionChanges > 3) {
                break;
            }

            ticker += dt.seconds();
            dt.reset();
        }

        setPowerDouble4(0, 0, 0, 0, 0);
    }

    public void rotatePID(double targetAngle, double rotationSpeed, int cycles, double p, double i, double d) {

//        rotationPID.start(3, 0.21, 0.69);
        rotationPID.start(p, i, d);
//        rotationPID.start(0.025, 0.005, 0);

        int ticker = 0;
        double startAngle = rotation;
        int directionChanges = 0;
        boolean lastPositiveState = true;

        while (ticker < cycles && Op.opModeIsActive()) {
            ticker++;
            double rotationPower = rotationPID.loop(targetAngle, rotation);
            rotationPower = rotationPower / (360);//rotationSpeed * Math.abs(startAngle - targetAngle));
            rotationPower += (0.01 * (rotationPower > 0 ? 1 : -1));
            Op.telemetry.addData("Error ", rotationPID.error);
            Op.telemetry.addData("Last Error  ", rotationPID.lastError);
            Op.telemetry.addData("Derivative ", rotationPID.derivative);
            Op.telemetry.addData("Integral ", rotationPID.integral);

            Op.telemetry.addData("TD ", rotationPID.deltaTime.seconds());

            Op.telemetry.addData("Rotation ", rotation);
            Op.telemetry.addData("rotationPower ", rotationPower);
            Op.telemetry.addData("rotationSpeed ", rotationSpeed);
            Op.telemetry.addData("yeets", directionChanges);
            Op.telemetry.update();
            rotateSimple(rotationPower * rotationSpeed);

            if (lastPositiveState != rotationPower > 0) {
                directionChanges++;
                lastPositiveState = rotationPower > 0;
            }

            if (directionChanges > 5) {
                break;
            }

        }

        setPowerDouble4(0, 0, 0, 0, 0);
    }


    public void rotateSimple(double targetAngle, double rotationSpeed, double tolerance, double exitTime) {
        double exitTimer = 0;
        ElapsedTime deltaTime = new ElapsedTime();


        while (Op.opModeIsActive()) {
            deltaTime.reset();

            moveComplex(new Double2(0, 0), rotationSpeed, getRotation() - targetAngle, 0);

            if (Math.abs(getRotation() - targetAngle) < tolerance) {
                exitTimer += deltaTime.seconds();
            }

            if (exitTimer < exitTime) {
                break;
            }
        }

        setPowerDouble4(0, 0, 0, 0, 0);
    }

    /**
     * @param v          this is the vector that represents our wheels power! Create a new Double4 like so:
     *                   new Double4(x,y,z,w)
     *                   <p>
     *                   See RobotConfiguration for more information
     * @param multiplier the coefficient of 'v'
     */

    public void setPowerDouble4(Double4 v, double multiplier) {
        driveManager.frontLeft.setPower(v.x * multiplier);
        driveManager.frontRight.setPower(v.y * multiplier);
        driveManager.backLeft.setPower(v.z * multiplier);
        driveManager.backRight.setPower(v.w * multiplier);
    }

    public void SetPersistentVector(Double2 vector, double imu) {

    }

    public void SetPersistentRotation(double relativeAngle) {
    }


    //Front left power is the X value
    //Front right power is the Y value
    //Back left power is the Z value
    //Back right power is the W value
    public void setPowerDouble4(double x, double y, double z, double w, double multiplier) {
        Double4 v = new Double4(x, y, z, w);

        driveManager.frontLeft.setPower(v.x * multiplier);
        driveManager.frontRight.setPower(v.y * multiplier);
        driveManager.backLeft.setPower(v.z * multiplier);
        driveManager.backRight.setPower(v.w * multiplier);

//        backRight.setPower(v.x * multiplier);
//        frontLeft.setPower(v.w * multiplier);
//        frontRight.setPower(v.z * multiplier);
//        backLeft.setPower(v.y * multiplier);
    }

    public void setDriveMode(DcMotor.RunMode mode) {
        driveManager.frontLeft.setMode(mode);
        driveManager.backLeft.setMode(mode);
        driveManager.frontRight.setMode(mode);
        driveManager.backRight.setMode(mode);
    }

    public void setRelativeEncoderPosition(double delta) {

        driveManager.frontLeft.setTargetPosition(driveManager.frontLeft.getCurrentPosition() + (int) delta);
        driveManager.backLeft.setTargetPosition(driveManager.backLeft.getCurrentPosition() + (int) delta);
        driveManager.frontRight.setTargetPosition(driveManager.frontRight.getCurrentPosition() + (int) delta);
        driveManager.backRight.setTargetPosition(driveManager.backRight.getCurrentPosition() + (int) delta);
    }

    public void setRelativeEncoderPosition(double deltaX, double deltaY, double deltaZ, double deltaW) {

        driveManager.frontLeft.setTargetPosition(driveManager.frontLeft.getCurrentPosition() + (int) deltaX);
        driveManager.backLeft.setTargetPosition(driveManager.backLeft.getCurrentPosition() + (int) deltaY);
        driveManager.frontRight.setTargetPosition(driveManager.frontRight.getCurrentPosition() + (int) deltaZ);
        driveManager.backRight.setTargetPosition(driveManager.backRight.getCurrentPosition() + (int) deltaW);
    }

    //Returns IMU rotation on the zed axies
    public double getRotation() {
        //returns the threaded rotation values for speeeed
        return rotation;
    }

    //Returns true if any wheels are currently busy
    public boolean wheelsBusy() {
        return driveManager.frontRight.isBusy() || driveManager.frontLeft.isBusy() || driveManager.backLeft.isBusy() || driveManager.backRight.isBusy();
//        return driveManager.frontRight.isBusy() || driveManager.frontLeft.isBusy() || driveManager.backLeft.isBusy() || driveManager.backRight.isBusy();
    }
    //</editor-fold>

    //Drive forward a set distance at a set speed, distance is measured in CM
    public void driveByDistance(double speed, double distance) {

        setDriveMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setRelativeEncoderPosition((480 / RobotConfiguration.wheel_circumference) * distance);
        setPowerDouble4(1, 1, 1, 1, speed);
        setDriveMode(DcMotor.RunMode.RUN_TO_POSITION);

        Op.telemetry.addData("Driving by distance ", distance * ((RobotConfiguration.wheel_circumference * RobotConfiguration.wheel_ticksPerRotation)));
        Op.telemetry.update();
        while (Op.opModeIsActive() && wheelsBusy()) {
            Op.telemetry.addData("Wheel Busy", "");
            Op.telemetry.addData("Wheel Front Right Postion", driveManager.frontRight.getCurrentPosition());
            Op.telemetry.addData("Wheel Front Right Target", driveManager.frontRight.motor.getTargetPosition());
            Op.telemetry.update();

            if (!Op.opModeIsActive()) {
                break;
            }
            //Wait until we are at our target distance
        }

        Op.telemetry.addData("Target Reached", "");
        Op.telemetry.update();

        //shutdown motors
        setPowerDouble4(0, 0, 0, 0, 0);

        //Set up for normal driving
        setDriveMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setDriveMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void driveByDistance(double angle, double speed, double distance) {

        setDriveMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        double distanceTicks = (480 / RobotConfiguration.wheel_circumference) * distance;
        Double4 a = bMath.getMecMovement(angle, 0, 0);

        setRelativeEncoderPosition(a.x * distanceTicks, a.y * distanceTicks, a.z * distanceTicks, a.w * distanceTicks);
        setPowerDouble4(1, 1, 1, 1, speed);

//        setRelativeEncoderPosition(a.x * distanceTicks, a.y * distanceTicks, a.z * distanceTicks, a.w * distanceTicks);
//        setPowerDouble4(a.x, a.y, a.z, a.w, speed);


        setDriveMode(DcMotor.RunMode.RUN_TO_POSITION);

        Op.telemetry.addData("Driving by distance ", distance * ((RobotConfiguration.wheel_circumference * RobotConfiguration.wheel_ticksPerRotation)));
        Op.telemetry.update();
        while (Op.opModeIsActive() && wheelsBusy()) {
            Op.telemetry.addData("Wheel Busy", "");
            Op.telemetry.addData("Wheel Front Right Postion", driveManager.frontRight.getCurrentPosition());
            Op.telemetry.addData("Wheel Front Right Target", driveManager.frontRight.motor.getTargetPosition());
            Op.telemetry.update();

            if (!Op.opModeIsActive()) {
                break;
            }
            //Wait until we are at our target distance
        }

        Op.telemetry.addData("Target Reached", "");
        Op.telemetry.update();

        //shutdown motors
        setPowerDouble4(0, 0, 0, 0, 0);

        //Set up for normal driving
        setDriveMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setDriveMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void driveByDistanceArmAsync(double angle, double speed, double distance, double targetArmDelay, double targetArmLength, double targetArmRotation) {

        setDriveMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        double distanceTicks = (480 / RobotConfiguration.wheel_circumference) * distance;
        Double4 a = bMath.getMecMovement(angle, 0, 0);

        setRelativeEncoderPosition(a.x * distanceTicks, a.y * distanceTicks, a.z * distanceTicks, a.w * distanceTicks);
        setPowerDouble4(1, 1, 1, 1, speed);

//        setRelativeEncoderPosition(a.x * distanceTicks, a.y * distanceTicks, a.z * distanceTicks, a.w * distanceTicks);
//        setPowerDouble4(a.x, a.y, a.z, a.w, speed);


        setDriveMode(DcMotor.RunMode.RUN_TO_POSITION);

        double runTime = 0;
        ElapsedTime deltaTime = new ElapsedTime();

        while (Op.opModeIsActive() && wheelsBusy() && !arm.armLengthTargetReached() && !arm.armRotationTargetReached()) {
            deltaTime.reset();

            if (runTime > targetArmDelay) {
                arm.setArmStateAsync(targetArmRotation, targetArmLength);
            }

            runTime += deltaTime.seconds();

            if (!Op.opModeIsActive()) {
                break;
            }
            //Wait until we are at our target distance
        }

        Op.telemetry.addData("Target Reached", "");
        Op.telemetry.update();

        //shutdown motors
        setPowerDouble4(0, 0, 0, 0, 0);

        //Set up for normal driving
        setDriveMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setDriveMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    @Deprecated
    public enum simpleDirection {
        FORWARD,
        BACKWARD,
        RIGHT,
        LEFT;
    }

    // can go forward, backwards, or sideways
    //distance should be in cm
    @Deprecated
    public void driveByDistancePoorly(double distance, simpleDirection direction, double speedMultiplier) {
        setDriveMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        int targetEncoders = (int) ((480.0 / RobotConfiguration.wheel_circumference) * distance);
        setDriveMode(DcMotor.RunMode.RUN_USING_ENCODER);

        if (direction == simpleDirection.FORWARD) { //if you wanna go forward, this is the stuff
            setPowerDouble4(1, 1, 1, 1, speedMultiplier);
            while (Op.opModeIsActive() && driveManager.backLeft.getCurrentPosition() < 0.5 * targetEncoders && driveManager.backRight.getCurrentPosition() < 0.5 * targetEncoders) {
            } //empty while loop works as waitUntil command
            setPowerDouble4(1, 1, 1, 1, 0.5 * speedMultiplier);
            while (Op.opModeIsActive() && driveManager.backLeft.getCurrentPosition() < 0.75 * targetEncoders && driveManager.backRight.getCurrentPosition() < 0.75 * targetEncoders) {
            } //empty while loop works as waitUntil command
            setPowerDouble4(1, 1, 1, 1, 0.25 * speedMultiplier);
            while (Op.opModeIsActive() && driveManager.backLeft.getCurrentPosition() < targetEncoders && driveManager.backRight.getCurrentPosition() < targetEncoders) {
            } //empty while loop works as waitUntil command
            setPowerDouble4(0, 0, 0, 0, 0);
        }

        if (direction == simpleDirection.LEFT) {
            setPowerDouble4(-1, 1, 1, -1, 1);
            while (driveManager.backLeft.getCurrentPosition() < 0.5 * targetEncoders && driveManager.backRight.getCurrentPosition() < 0.5 * targetEncoders) {
            } //empty while loop works as waitUntil command
            setPowerDouble4(-1, 1, 1, -1, 0.5);
            while (driveManager.backLeft.getCurrentPosition() < 0.75 * targetEncoders && driveManager.backRight.getCurrentPosition() < 0.75 * targetEncoders) {
            } //empty while loop works as waitUntil command
            setPowerDouble4(-1, 1, 1, -1, 0.25);
            while (driveManager.backLeft.getCurrentPosition() < targetEncoders && driveManager.backRight.getCurrentPosition() < targetEncoders) {
            } //empty while loop works as waitUntil command
            setPowerDouble4(-1, 1, 1, -1, 0);
        }

        if (direction == simpleDirection.BACKWARD) ;
        {
            setPowerDouble4(-1, -1, -1, -1, 1);
            while (driveManager.backLeft.getCurrentPosition() < 0.5 * targetEncoders && driveManager.backRight.getCurrentPosition() < 0.5 * targetEncoders) {
            } //empty while loop works as waitUntil command
            setPowerDouble4(-1, -1, -1, -1, 0.5);
            while (driveManager.backLeft.getCurrentPosition() < 0.75 * targetEncoders && driveManager.backRight.getCurrentPosition() < 0.75 * targetEncoders) {
            } //empty while loop works as waitUntil command
            setPowerDouble4(-1, -1, -1, -1, 0.25);
            while (driveManager.backLeft.getCurrentPosition() < targetEncoders && driveManager.backRight.getCurrentPosition() < targetEncoders) {
            } //empty while loop works as waitUntil command
            setPowerDouble4(-1, -1, -1, -1, 0);
        }

        if (direction == simpleDirection.RIGHT) ;
        {
            setPowerDouble4(1, -1, -1, 1, 1);
            while (driveManager.backLeft.getCurrentPosition() < 0.5 * targetEncoders && driveManager.backRight.getCurrentPosition() < 0.5 * targetEncoders) {
            } //empty while loop works as waitUntil command
            setPowerDouble4(1, -1, -1, 1, 0.5);
            while (driveManager.backLeft.getCurrentPosition() < 0.75 * targetEncoders && driveManager.backRight.getCurrentPosition() < 0.75 * targetEncoders) {
            } //empty while loop works as waitUntil command
            setPowerDouble4(1, -1, -1, 1, 0.25);
            while (driveManager.backLeft.getCurrentPosition() < targetEncoders && driveManager.backRight.getCurrentPosition() < targetEncoders) {
            } //empty while loop works as waitUntil command
            setPowerDouble4(1, -1, -1, 1, 0);
        }
    }


    //This version of drive by distance doesnt use Drive To Position and keeps the oriantation the same
    //WIP

    //3666 X
    //11577
    //16884
//    public void driveByDistance(double speed, double distance, double targetRotation) {
//
//        double distances = (480 / RobotConfiguration.wheel_circumference) * distance;
//        Double4 targetDistances = new Double4(distances, distances, distances, distances);
//
//        setPowerDouble4(1, 1, 1, 1, speed);
//        while (op.opModeIsActive() && driveManager.backRight > targetDistances.x) {
//            setPowerDouble4();
//
//
//            if (!op.opModeIsActive()) {
//                break;
//            }
//        }
//
//        op.telemetry.addData("Target Reached", "");
//        op.telemetry.update();
//
//        //shutdown motors
//        setPowerDouble4(0, 0, 0, 0, 0);
//
//        //Set up for normal driving
//        setDriveMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        setDriveMode(DcMotor.RunMode.RUN_USING_ENCODER);
//    }

    //Returns the distance using a sensor group
    public double getDistance(RobotWallTrack.groupID group, DistanceUnit unit) {
        return wallTrack.sensorIDGroupPairs.get(group).getDistanceAverage(unit);
    }

    //Sets the position of the foundation 'arm' servos,
    //Setting this value to 0 will raise the arms all of the way
    //Setting this value to 0.73 will lower the arms without impacting wheels
    //Setting this value to 1 will lower the arms and strain on the wheels, should only be used when dragging the foundation
    public void setFoundationGripperState(double value) {
        foundationServo0.setPosition(1 - value);
        foundationServo1.setPosition(value);
    }

    //grips the foundation, meant to be human readable (by judges)
    public void gripFoundation() {
        setFoundationGripperState(0);
    }

    //lets go of the foundation, meant to be human readable (by judges)
    public void releaseFoundation() {
        setFoundationGripperState(0.9);
    }
}
