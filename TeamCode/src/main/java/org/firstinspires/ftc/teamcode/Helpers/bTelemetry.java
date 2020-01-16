package org.firstinspires.ftc.teamcode.Helpers;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

public class bTelemetry {
    static OpMode opMode;

    public static void start(OpMode op) {
        opMode = op;
        print("Started.");
    }

    //Adds one line without clearing the log
    public static void print(String message) {
        opMode.telemetry.log().add("bTelemetry: " + message);
        opMode.telemetry.update();
    }

    //Adds one line without clearing the log
    public static void print(String caption, Object data) {
        opMode.telemetry.addData("bTelemetry: " + caption, data.toString());
        opMode.telemetry.update();
    }

}
