package frc.robot.utility;

public class MathEqs {

    public static double targetLinear(double offset, double baseline, double upperBound, double lowerBound) {
        return (offset < upperBound) ? ((baseline * offset) / (upperBound - lowerBound)
                + (baseline * lowerBound) / (lowerBound - upperBound)) : baseline;
    }

    public static double targetLinear2(double offset, double maximum, double minimum, double upperBound, double lowerBound) {
        return (offset < upperBound) ? ((maximum - minimum) * offset / (upperBound - lowerBound)
                + (maximum * lowerBound) / (lowerBound - upperBound)) + minimum : maximum;
    }

    public static double targetQuadratic(double a, double b, double c, double x) {
        return a*x*x + b*x + c;
    }

    public static double roundCustom(double x) {return Math.round(x*1000.0)/1000.0;}

}