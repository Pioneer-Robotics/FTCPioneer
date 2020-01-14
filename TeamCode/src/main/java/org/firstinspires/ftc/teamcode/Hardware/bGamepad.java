package org.firstinspires.ftc.teamcode.Hardware;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.Helpers.TriggerBoolean;
import org.firstinspires.ftc.teamcode.Helpers.Vector2;

public class bGamepad {

    TriggerBoolean aTb, bTb, xTb, yTb;
    public boolean aTrigger, bTrigger, xTrigger, yTrigger;

    Vector2 getJoystick_leftCache;

    public Vector2 joystick_left() {
        getJoystick_leftCache.x = gamepad.left_stick_x;
        getJoystick_leftCache.y = gamepad.left_stick_x;
        return getJoystick_leftCache;
    }


    Vector2 joystick_rightCache;

    public Vector2 joystick_right() {
        joystick_rightCache.x = gamepad.right_stick_x;
        joystick_rightCache.y = gamepad.right_stick_y;
        return joystick_rightCache;
    }

    public boolean dpad_up = false;
    public boolean dpad_right = false;
    public boolean dpad_left = false;
    public boolean dpad_down = false;

    public boolean start;
    public boolean guide;
    public boolean back;

    //Dead zone is set low to allow for more granular controls
    public float deadZone = 0.025f;

    public Gamepad gamepad;

    public bGamepad() {


        aTb = new TriggerBoolean();
        bTb = new TriggerBoolean();
        xTb = new TriggerBoolean();
        yTb = new TriggerBoolean();
    }

    public void Update() {
        gamepad.setJoystickDeadzone(deadZone);

        aTb.assign(gamepad.a);
        bTb.assign(gamepad.b);
        xTb.assign(gamepad.x);
        yTb.assign(gamepad.y);

        dpad_down = gamepad.dpad_down;
        dpad_up = gamepad.dpad_up;
        dpad_right = gamepad.dpad_right;
        dpad_left = gamepad.dpad_left;

        aTrigger = aTb.getTrigger();
        bTrigger = bTb.getTrigger();
        xTrigger = xTb.getTrigger();
        yTrigger = yTb.getTrigger();
    }

}
