package org.firstinspires.ftc.teamcode.Helpers;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.TeleOp.DriverControls.EngineeringControlData;

public class PositionTracker {
    double waitInterval;
    //waitInterval is the time (in seconds) you want the computer to wait between getting data from the odometry wheels
    //the program probably works best when it's set close to 0, but the exact best number should be found through testing
    //the value of this will introduce some of the error
    //TODO find the optimal value
    double timeWaited; //used to track time passed since values were pulled from the odometry wheels

    double xPos; //the robot's x coordinate
    double yPos; //the robot's y coordinate
    double orientation; //the robot's rotation (in radians)

    double frontLeftLast;
    double frontRightLast;
    double backLast;

    ElapsedTime deltaTime = new ElapsedTime();

    public PositionTracker(double waitInterval){
        this.waitInterval = waitInterval;
        double xPos = 0;
        double yPos = 0;
        double orientation = 0;
    }

    //calculatePosition updates the variables in the PositionTracker object
    private void calculatePosition (){
        timeWaited += deltaTime.seconds();

        if(timeWaited >= waitInterval){ //if you've waited long enough
            timeWaited = 0; //we pull the values, so start tracking time till next cycle
            double deltafrontLeft = getFLOdometer() - frontLeftLast;
            double deltafrontRight = getFROdometer() - frontRightLast;
            double deltaBack = getRearOdometer() - backLast;

            setLastValuesToCurrent(); //sets frontLeftLast, frontRightLast, and backLast to the current odometer values

            /*
            calculateNewPosition takes the change in odometer values and
            calculates the change in the robot's position & orientation
             */
            calculateNewPosition(deltafrontLeft, deltafrontRight, deltaBack);
        }
        //otherwise, just keep waiting
    }

    //getPosition creates a Position object with current info
    Position getPosition(){
        Position pos = new Position(xPos,yPos,orientation);
        return pos;
    }

    //showPosition prints what getPosition thinks the position is to the control phone's screen so we can test it
    void showPosition(){

    }

    void calculateNewPosition(double deltaFL, double deltaFR, double deltaBack){
        double radiansTurned = (deltaFL - deltaFR) / EngineeringControlData.gapWidth;
        //if the robot turns exactly about the centerpoint, this won't work
        //if the robot rotates about a point not on the centerline, this won't work (hint: need 3rd wheel to find)
        //TODO fix the above bugs

        if(radiansTurned == 0) {
            //that would mean it moved in a straight line
            xPos += deltaBack * Math.cos(orientation);
            xPos += deltaFL * Math.sin(orientation); //it didn't turn, so deltaFL and deltaFR are equal

            yPos += deltaBack * Math.sin(orientation);
            yPos += deltaFL * Math.cos(orientation);
        }

        else{ //that means it turned, and finding the position then becomes much more complicated

            findPosChangeBasedOnFrontWheels(deltaFL / 2, deltaFR / 2, radiansTurned / 2);
            findPosChangeBasedOnBackWheel(deltaBack, radiansTurned);
            findPosChangeBasedOnFrontWheels(deltaFL / 2, deltaFR / 2, radiansTurned / 2);
            /* because I don't yet know how to consider both the "front" wheels (the two that roll front to back)
            and the "back" wheel (which rolls side to side) at the same time, I'm first doing half the front wheels,
            then all the back wheel, then the other half of the front wheels
             */
        }
    }

    private void findPosChangeBasedOnFrontWheels(double deltaFL, double deltaFR, double radiansTurned){
        boolean counterClockwise;
        if(deltaFR > deltaFL){
            counterClockwise = true;
        }
        else{
            counterClockwise = false;
        }

        double a; //a is the same as the gapWidth, but sometimes it needs to be negative to make the math work

        if(counterClockwise) {
            a = EngineeringControlData.gapWidth;
        }
        else{
            a = EngineeringControlData.gapWidth * (-1);
        }

        double turnRadius; //this is the radius of the turn made by the point between exactly the FL and FR wheels
        turnRadius = a * deltaFL / (deltaFR - deltaFL) + EngineeringControlData.gapWidth;

        double relXChange = turnRadius * Math.cos(radiansTurned); //relative to current position and rotation
        double relYChange = turnRadius * Math.sin(radiansTurned);

        double distance = Math.sqrt( bMath.squared(relXChange) + bMath.squared(relYChange) );

        orientation += radiansTurned;

        relXChange = distance * Math.cos(orientation); //relative to current position and absolute rotation
        relYChange = distance * Math.sin(orientation);

        xPos += relXChange;
        yPos += relYChange;
    }

    private void findPosChangeBasedOnBackWheel(double deltaBack, double radiansTurned){
        //adjust for the amount the back wheel spins just from rotation
        deltaBack -= EngineeringControlData.yOffSet * radiansTurned;

        //find effect of "sideways" motion on our total position
        double relXChange = deltaBack * Math.cos(orientation); //relative to current position and absolute rotation
        double relYChange = deltaBack * Math.sin(orientation); //same
        xPos += relXChange;
        yPos += relYChange;
    }

    //condensing 3 lines of code into 1 line for use elsewhere
    private void setLastValuesToCurrent() {
        frontLeftLast = getFLOdometer();
        frontRightLast = getFROdometer();
        backLast = getRearOdometer();
    }

    //these are just placeholder methods until we get the actual odometry wheels installed and can get values from them
    //TODO get the actual odometers installed and connected so these methods can be replaced
    double getFLOdometer(){
        return 5.0;
    }
    double getFROdometer(){
        return 5.0;
    }
    double getRearOdometer(){
        return 5.0;
    }

}
