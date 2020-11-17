package org.firstinspires.ftc.teamcode.Helpers;

//this class is gonna deal with complex numbers, probably will make "Position" class obsolete
public class ComplexNum {
    double real; //real component
    double imag; //imaginary component

    public ComplexNum(){ //just here so using the default constructor is still possible

    }
    public ComplexNum(double real, double imag){
        this.real = real;
        this.imag = imag;
    }
    public static ComplexNum add(ComplexNum a, ComplexNum b){
        ComplexNum Ans = new ComplexNum();
        Ans.real = a.real + b.real;
        Ans.imag = a.imag + b.imag;
        return Ans;
    }
    public static ComplexNum multiply(ComplexNum a, ComplexNum b){
        ComplexNum Ans = new ComplexNum();
        Ans.real = (a.real * b.real) - (a.imag * b.imag);
        Ans.imag = (a.imag * b.real) + (a.real * b.imag);
        return Ans;
    }
    public void plusEquals(ComplexNum input){
        this.equals(add(this,input));
    }

    //changes the ComplexNum object to have values equal to the input
    public void equals(ComplexNum input){
        real = input.real;
        imag = input.imag;
    }

    public ComplexNum clone(){
        return new ComplexNum(this.real, this.imag);
    }

}
