package org.firstinspires.ftc.teamcode.Hardware;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.Helpers.TriggerBoolean;

public class bGamepad {

    public TriggerBoolean a, b, x, y;



    public Gamepad gamepad;

    public bGamepad() {
        a = new TriggerBoolean();
        b = new TriggerBoolean();
        x = new TriggerBoolean();
        y = new TriggerBoolean();
    }

    public void Update() {
        a.assign(gamepad.a);
        b.assign(gamepad.b);
        x.assign(gamepad.x);
        y.assign(gamepad.y);
    }



}
