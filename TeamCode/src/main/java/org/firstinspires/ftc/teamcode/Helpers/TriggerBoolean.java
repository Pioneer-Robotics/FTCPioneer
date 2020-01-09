package org.firstinspires.ftc.teamcode.Helpers;

public class TriggerBoolean {

    //The current state of this var
    boolean state;

    //The last state of the state
    boolean lastState;

    //Set to true after the value of state changes
    private boolean trigger;

    public boolean getTrigger() {
        return state != lastState;
    }

    public void Assign(boolean newState) {
        lastState = state;
        state = newState;
    }

    public boolean Value() {
        return state;
    }

}
