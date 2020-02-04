package org.firstinspires.ftc.teamcode.TeleOp.DriverControls;

public class EngineeringControlData {
    //Arm movement data
    public boolean powerExtension = true;
    public double extension = 0;
    public double extendSpeed = 0;
    public double raiseSpeed = 0;
    //rectangular control state
    public boolean rectControls = false;
    public boolean rectControlsCheck = false;
    public boolean rectControls_goingUp = false;
    public boolean rectControls_goingUpCheck = false;
    //rect control constants
    public double xExtConst = 1;
    public double yExtConst = 1;
}