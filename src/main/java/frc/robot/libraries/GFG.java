/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.libraries;

public class GFG{ 
    static final float EPSILON = (float)0.01; 
  
    // An example function whose solution is determined using 
    // Bisection Method. The function is x^3 - x^2  + 2 
    static double func(double x,double[] coefficient) 
    { 
        double temp = 0;
        double exp = 0;
        for(double a:coefficient){
            temp+=a*Math.pow(x, exp);
            exp++;
        }
        return temp; 
    } 
  
    // Prints root of func(x) with error of EPSILON 
    public static double bisection(double a, double b,double[] coefficient) 
    { 
        if (func(a,coefficient) * func(b,coefficient) >= 0) 
        { 
            System.out.println("You have not assumed"
                        + " right a and b"); 
            return 0; 
        } 
  
        double c = a; 
        while ((b-a) >= EPSILON) 
        { 
            // Find middle point 
            c = (a+b)/2; 
  
            // Check if middle point is root 
            if (func(c,coefficient) == 0.0) 
                break; 
  
            // Decide the side to repeat the steps 
            else if (func(c,coefficient)*func(a,coefficient) < 0) 
                b = c; 
            else
                a = c; 
        } 
                //prints value of c upto 4 decimal places 
        return c;
    } 
    // public static void main(String[] args) 
    // { 
    //      Initial values assumed 
    //     double a =-200, b = 300; 
    //     bisection(a, b); 
    // }
    // Driver program to test above function 
    // This code is contributed by Nirmal Patel 
} 