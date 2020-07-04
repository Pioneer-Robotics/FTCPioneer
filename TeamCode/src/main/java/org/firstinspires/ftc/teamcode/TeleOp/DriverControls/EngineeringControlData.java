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
    //TODO find these actual values, rn they are all placeholders
    public static double gapWidth = 10.0; //gap between the 2 front/back wheels
    public static double ticsToCM = 23.2; //the ratio between cm the wheel has travelled and the ticks counted by encoder

}