package org.firstinspires.ftc.teamcode.TeleOp.DriverControls;

public class EngineeringControlData {
    //Arm movement data
    public boolean powerExtension = true;
    public double extension = 0;
    public double extendSpeed = 0;
    public double raiseSpeed = 0;
    //spool protection override
    public boolean spoolProtect = true;
    public boolean spoolProtectCheck = false;
    //rectangular control state
    public boolean rectControls = false;
    public boolean rectControlsCheck = false;
    public boolean rectControls_goingUp = false;
    public boolean rectControls_goingUpCheck = false;
    //rect control constants
    public double xExtConst = 1;
    public double yExtConst = 1;
    //odometery info
    //TODO find the actual gapWidth, rn it's a placeholder
    public static double gapWidth = 10.0; //gap between the 2 front/back wheels
    public static double wheelDiameter = 5.08; //that's cm
    public static int ticksPerRotation = 8192;
    public static double ticsToCM = wheelDiameter * Math.PI / ticksPerRotation;

    // yOffSet = the distance from the wheel that moves left/right to the axle connecting the other wheels
    public static double yOffSet = 0.0;

}