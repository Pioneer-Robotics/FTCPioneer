package org.firstinspires.ftc.teamcode.Helpers;

import org.firstinspires.ftc.teamcode.TeleOp.DriverControls.EngineeringControlData;

//the purpose of objects of this class is to track the position of the point exactly between
//the two forward facing odometry wheels relative to the "starting" position and orientation
//"starting" refrence point can be anywhere on the mat, probably will not be where the robot starts
public class PositionTracker {
    private double rotation; //the robot's rotation (in radians)

    private ComplexNum position; //real part is like x, imag part is like y

    private double leftLast;
    private double rightLast;
    private double middleLast;
    private double rotationLast;

    private double deltaLeft;
    private double deltaRight;
    private double deltaMiddle;
    private double deltaRotation;

    public PositionTracker(double x, double y){
        rotation = 0;
        position = new ComplexNum(x,y);
    }
    public PositionTracker(double x, double y, double theta){
        rotation = theta;
        position = new ComplexNum(x,y);
    }

    public void update(){ //order matters here, don't change em round
        calculateDeltas();
        adjustPosition();
        calculateRotation();
        setLastValuesToCurrent();
    }
    public ComplexNum getPosition(){
        return position.clone();
    }
    public double getRotation(){
        return rotation; //we do not calculateRotation() first ON PURPOSE
    }
    private void calculateRotation(){
        rotation = ( getRightOdometer() - getLeftOdometer() ) / EngineeringControlData.gapWidth;
    }

    private void calculateDeltas(){ //finds the change in all the odometry readings, and calculates rotation change
        deltaLeft = getLeftOdometer() - leftLast;
        deltaRight = getRightOdometer() - rightLast;
        deltaMiddle = getMiddleOdometer() - middleLast;
        deltaRotation = (deltaRight - deltaLeft) / EngineeringControlData.gapWidth;
    }

    private void adjustPosition(){
        // find displacement without worrying about needing to correct for rotation
        ComplexNum displacement = calculateDisplacement();
        // rotate displcement to match up with the starting axis
        rotateComplexNum(displacement, rotationLast);
        // add rotated displacement to the previous position value, that is your new position
        position.plusEquals(displacement);
    }

    //calculates displacement relative to its "last" position and rotation as a ComplexNum
    private ComplexNum calculateDisplacement(){
        //TODO go through this step by step and explain what's going on (in comments)
        //TODO: rn, this doesn't use the middle odometer at all, fix that.
        //TODO: this assumes it rotates about a point in line w/ the odometers, that's not true
        double arcLength = (deltaRight + deltaLeft) / 2;
        double turnRadius = deltaRotation * arcLength;
        double halfTheta = deltaRotation / 2.0;
        double chordLength = 2 * Math.sin(halfTheta) * turnRadius;
        double xDisplacement = chordLength * Math.sin(halfTheta) * -bMath.sign(deltaRotation);
        double yDisplacement = chordLength * Math.cos(halfTheta) * bMath.sign(deltaRotation);
        return new ComplexNum(xDisplacement, yDisplacement);
    }

    //condensing 3 lines of code into 1 line for use elsewhere
    private void setLastValuesToCurrent() {
        leftLast = getLeftOdometer();
        rightLast = getRightOdometer();
        middleLast = getMiddleOdometer();
        rotationLast = rotation;
    }

    //this changes the data in the ComlexNum "num" so it's rotated by the angle
    private void rotateComplexNum(ComplexNum num, double angle){
        ComplexNum newComplex = ComplexNum.multiply(num, bMath.cis(angle));
        num.equals(newComplex);
    }

    //these are just placeholder methods until we get the actual odometry wheels installed and can get values from them
    //TODO get the actual odometers installed and connected so these methods can be replaced
    double getLeftOdometer(){
        return 5.0;
    }
    double getRightOdometer(){
        return 5.0;
    }
    double getMiddleOdometer(){
        return 5.0;
    }

}
