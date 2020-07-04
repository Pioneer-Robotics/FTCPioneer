package org.firstinspires.ftc.teamcode.Helpers;

public class Position {
    /*the point of this class is to make objects that can hold info
    on the robot's x coordinate, y coordinate, and orientation
     */
    double xCoordinate;
    double yCoordinate;
    double rotation;

    public Position(double xPos, double yPos, double orientation){
        xCoordinate = xPos;
        yCoordinate = yPos;
        rotation = orientation;
    }
}
