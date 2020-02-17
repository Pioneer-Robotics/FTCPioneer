package org.firstinspires.ftc.teamcode.Helpers;

public class Vector2 {
    public double x;
    public double y;

    public Vector2(double _x, double _y) {
        x = _x;
        y = _y;
    }

    //Creates a Vector2 from an angle in DEGREES
    public Vector2(double degrees) {
        x = Math.cos(degrees);
        y = Math.sin(degrees);
    }

    public void add(Vector2 value) {
        x += value.x;
        y += value.y;
    }

    public void multiply(Double value){
        x = x*value;
        y = y*value;

    }

    public void subtract(Vector2 value) {
        x -= value.x;
        y -= value.y;
    }


    public double magnitude(){
        return Math.sqrt(x*x + y*y);
    }

    //soh cah toa
    public double angle(){
        return Math.atan2(y,x);
    }

    public void changeAngle(double angle){
        double mag = magnitude();
        x = Math.cos(angle*mag);
        y = Math.sin(angle*mag);
    }

}


