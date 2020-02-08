package org.firstinspires.ftc.teamcode.TeleOp;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.TimestampedI2cData;
import com.qualcomm.robotcore.util.RobotLog;
import com.qualcomm.robotcore.util.ThreadPool;

import org.firstinspires.ftc.robotcore.internal.opmode.TelemetryInternal;
import org.firstinspires.ftc.robotcore.internal.opmode.OpModeManagerImpl;
import org.firstinspires.ftc.teamcode.Hardware.bGamepad;

import java.util.concurrent.CancellationException;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.TimeUnit;

public abstract class TeleOpMode extends LinearOpMode {

    public bGamepad driverController;
    public bGamepad engineeringController;

    public void initControllers() {
        driverController = new bGamepad(gamepad1);
        engineeringController = new bGamepad(gamepad2);
    }

    public void updateControllers() {
        driverController.Update();
        engineeringController.Update();
    }

}