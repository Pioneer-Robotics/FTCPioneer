package org.firstinspires.ftc.teamcode.Robot;

import android.renderscript.Double2;

//This class has all of the names and data for all of the things in one place for easy of access / sanity!
public class RobotConfiguration {

    //The IMU's names
    public static final String imu_0 = "imu";
    public static final String imu_1 = "imu1";

    public static final String camera = "Webcam 1";

    /* Our cool robot configuration


         0
      ________
    |X|      |Y|
    | |  ^^  | |
270 | |      | |   90
    |Z|______|W|

        180


    Wheel placement and angles verified by Ben on the 21nd of November

     */


    //Configuration names for all of our wheels
    public static final String wheel_frontLeft = "Front Left";//This wheel should correspond to the X component of movement
    public static final String wheel_frontRight = "Front Right";//This wheel should correspond to the Y component of movement
    public static final String wheel_backLeft = "Back Left";//This wheel should correspond to the Z component of movement
    public static final String wheel_backRight = "Back Right";//This wheel should correspond to the Z component of movement


    //The amount of encoder ticks per motor rotation
    public static final int wheel_ticksPerRotation = 1440;

    //The max speed our wheel motors will ever rotate (in ticks per second), 3 rotations per second. Used in calibration.
    public static final int wheel_maxTicksPerSecond = 4320;

    //pi * diameter
    public static final double wheel_circumference = 32.2;
}
