package frc.robot.utility;

public class MathEqs {

    public static double targetLinear(double offset, double baseline, double upperBound, double lowerBound) {
        return (offset < upperBound) ? ((baseline * offset) / (upperBound - lowerBound)
                + (baseline * lowerBound) / (lowerBound - upperBound)) : upperBound;
    }

}