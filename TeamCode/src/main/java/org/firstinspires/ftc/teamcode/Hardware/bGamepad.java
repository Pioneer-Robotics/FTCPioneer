package org.firstinspires.ftc.teamcode.Hardware;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.Helpers.TriggerBoolean;
import org.firstinspires.ftc.teamcode.Helpers.Vector2;

public class bGamepad {

    TriggerBoolean aTb, bTb, xTb, yTb;
    public boolean a, b, x, y;
    public boolean aTriggered, bTriggered, xTriggered, yTriggered;

    Vector2 getJoystick_leftCache;

    public Vector2 leftJoystick() {
        getJoystick_leftCache.x = gamepad.left_stick_x;
        getJoystick_leftCache.y = gamepad.left_stick_x;
        return getJoystick_leftCache;
    }


    Vector2 joystick_rightCache;

    public Vector2 rightJoystick() {
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


    public boolean rightBumper;
    public float rightTrigger;

    public boolean leftBumper;
    public float leftTrigger;

    //Dead zone is set low to allow for more granular controls
    public float deadZone = 0.025f;

    Gamepad gamepad;

    public bGamepad(Gamepad base) {
        gamepad = base;

        aTb = new TriggerBoolean();
        bTb = new TriggerBoolean();
        xTb = new TriggerBoolean();
        yTb = new TriggerBoolean();
    }

    public void Update() {
        gamepad.setJoystickDeadzone(deadZone);

        a = gamepad.a;
        b = gamepad.b;
        x = gamepad.x;
        y = gamepad.y;

        aTb.assign(a);
        bTb.assign(b);
        xTb.assign(x);
        yTb.assign(y);

        guide = gamepad.guide;
        start = gamepad.start;
        back = gamepad.back;

        dpad_down = gamepad.dpad_down;
        dpad_up = gamepad.dpad_up;
        dpad_right = gamepad.dpad_right;
        dpad_left = gamepad.dpad_left;

        aTriggered = aTb.getTrigger();
        bTriggered = bTb.getTrigger();
        xTriggered = xTb.getTrigger();
        yTriggered = yTb.getTrigger();

        rightBumper = gamepad.right_bumper;
        rightTrigger = gamepad.right_trigger;

        leftBumper = gamepad.left_bumper;
        leftTrigger = gamepad.left_trigger;
    }

}
