// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.vision;

/** Calculates the result of a polynomial function with x and an array of coefficients (IN ORDER OF DEGREE)*/
public class PolynomialFunction {
    public double polynomailFunction (double x, double[] coefficients) {
        double value = 0;
         Integer i = 0;
         for (double coefficient : coefficients) {
            if (i>0) {
                value = value + Math.pow(coefficient*x, i);
            }
            else {
                value = value + coefficient;
            }
            i++;
         }
         return value;
    }
}
