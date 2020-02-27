package org.firstinspires.ftc.teamcode.Robot;

import java.util.HashMap;
import java.util.Map;

//Shared robot related functions that can be identified by an enum, far from the most elegant solution. The original idea was to use lambdas, java had other plans
//All functions must adhere to a loose functional pattern where values are cached to avoid GC alloc,

//WIPWIPWIWPIWPWIPWIWP
public class SharedFunctions {

    public enum Function {
        openArmGripper
    }

    public static Map<Function, Action> functionRunnableDictionary = new HashMap<Function, Action>();

    public static void start(final Robot inputRobot) {
    }

    public static void execute(Function function) {
        functionRunnableDictionary.get(function).run();
    }

    public static void execute(Function function, double value) {
        functionRunnableDictionary.get(function).run(value);
    }
}


abstract class Action {

    public void run() {
    }

    public void run(double value) {
    }

}
