package frc.robot.utility;

import frc.robot.Constants;

public class MathEqs {

    public static float targetLinear(float baseline, float offset, float upperBound, float lowerBound) {
        return ((baseline * Math.abs(offset)) / (upperBound - lowerBound)
                + (baseline * lowerBound) / (lowerBound - upperBound));
    }

}