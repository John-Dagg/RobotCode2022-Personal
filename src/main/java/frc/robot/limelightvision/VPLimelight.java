package frc.robot.limelightvision;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.utility.MathEqs;

import static frc.robot.Constants.LimelightVision.HighGear.*;
import static frc.robot.Constants.LimelightVision.HighGear.maxTurn_High;
import static frc.robot.Constants.LimelightVision.LowGear.*;
import static frc.robot.Constants.LimelightVision.LowGear.maxTurn_Low;

public class VPLimelight extends SubsystemBase {

    private NetworkTable mNetworkTable;

    private NetworkTableEntry vpTargets, vpxOffset, vpyOffset, vpTargetArea, vpTargetSkew;
    private double targets, xOffset, yOffset, targetArea, targetSkew;

    public double throttleValue, turnValue;
    double startTime;
    double elapsedTime;

    public VPLimelight(){
        mNetworkTable = NetworkTableInstance.getDefault().getTable("limelight");

        //printNetworkTables();
    }

    @Override
    public void periodic(){
//        System.out.println("Distance Calc: " + MathEqs.roundCustom(calcDistance()));
    }

    public void printNetworkTables(){

        updateTargets();

        SmartDashboard.putNumber("Targets Found", targets);
        SmartDashboard.putNumber("Horizontal Offset", xOffset);
        SmartDashboard.putNumber("Vertical Offset", yOffset);
        SmartDashboard.putNumber("Area of Target", targetArea);
        SmartDashboard.putNumber("Target Skew/Tilt", targetSkew);

        System.out.println("Targets Found: " + targets);
        System.out.println("Horizontal Offset: " + xOffset);
        System.out.println("Vertical Offset: " + yOffset);
        System.out.println("Area of Target: " + targetArea);
        System.out.println("Target Skew/Tilt" + targetSkew);

    }

    public void updateTargets(){
        vpTargets = mNetworkTable.getEntry("tv");
        vpxOffset = mNetworkTable.getEntry("tx");
        vpyOffset = mNetworkTable.getEntry("ty");
        vpTargetArea = mNetworkTable.getEntry("ta");
        vpTargetSkew = mNetworkTable.getEntry("ts");

        targets = vpTargets.getDouble(0.0);
        xOffset = vpxOffset.getDouble(0.0);
        yOffset = vpyOffset.getDouble(0.0);
        targetArea = vpTargetArea.getDouble(0.0);
        targetSkew = vpTargetSkew.getDouble(0.0);
    }

    public double getTargets(){return targets;}
    public double getxOffset(){return xOffset;}
    public double getyOffset(){return yOffset;}
    public double getTargetArea(){return targetArea;}
    public double getTargetSkew(){return targetSkew;}

    public void flashArray(){
        NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(2);
        //Flashes limelight's lights
    }

    public void offArray(){
        NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(1);
    }

    public void steadyArray(){
        NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(0);
        //Set the limelight's array to the settings in the pipeline

    }

    public void setValues(double throttle, double turn) {
        throttleValue = throttle;
        turnValue = turn;
    }

    public double[] getValues() {
        return new double[] {throttleValue, turnValue};
    }

    public double calcDistance(){
        steadyArray();
        updateTargets();
        yOffset = getyOffset();
//        System.out.println(yOffset);
        offArray();
        return (Constants.LimelightVision.targetHeight - Constants.LimelightVision.cameraHeight)
                / Math.tan(Units.degreesToRadians(Constants.LimelightVision.cameraAngle + yOffset));
    }

    public boolean aimTarget(Drivetrain mDrivetrain, Constants.DriveTrain.DriveState limelightMode, double start, double bufferTime){
        double turn = 0;
        double yaw = getxOffset();
//        System.out.println(mDrivetrain.getShiftState());

        if (mDrivetrain.getShiftState() != Constants.DriveTrain.ShiftState.HIGH_GEAR) {
            if (deadbandAngle_Low < Math.abs(yaw)) {
                /***
                 deadband angle is the acceptable offset from what is supposed to be the center of the target
                 deccel angle is the angle the robot starts decelerating at
                 max turn is the highest speed the robot will turn at
                 yaw is the angle the robot is away from the center of the target
                 */

                turn = Math.signum(yaw) * MathEqs.targetLinear2(Math.abs(yaw), maxTurn_Low, minturn_Low, deccelAngle_Low, deadbandAngle_Low);
//                System.out.println("Yaw: " + yaw + " Turn: " + turn);
            }
        } else {
            if (deadbandAngle_High < Math.abs(yaw)) {
                turn = Math.signum(yaw) * MathEqs.targetLinear2(Math.abs(yaw), maxTurn_High, minturn_High, deccelAngle_High, deadbandAngle_High);
//                turn = Math.signum(yaw) * MathEqs.targetLinear2(Math.abs(yaw), maxTurn_Low, minturn_Low, deccelAngle_Low, deadbandAngle_Low);
//                System.out.println("Yaw: " + yaw + " Turn: " + turn);
//                System.out.println("WHAT IS HAPPENING...");
            }
        }
        if (limelightMode == Constants.DriveTrain.DriveState.AUTO_LIMELIGHT && Math.abs(turn) < 0.25) {
            if (startTime == -1) {
                System.out.println("RESET: "+startTime);
                startTime = System.currentTimeMillis();
                System.out.println("START TIME "+startTime);
            }

            elapsedTime = Math.abs((System.currentTimeMillis() - startTime) /1000);
            System.out.println("TIME REMAINING: "+(bufferTime-elapsedTime));
//            System.out.println("Please work");
            if (elapsedTime >= 0.5) {
                System.out.println("STOPPING ALIGNMENT");
                return true;
            }
        }
        else {
            startTime = -1;
            elapsedTime = 0;
        }
        setValues(0, turn);
        return false;
    }

    public void findTarget(Drivetrain mDrivetrain, double searchDirection){
        if (mDrivetrain.getShiftState() != Constants.DriveTrain.ShiftState.HIGH_GEAR){
            setValues(0, searchDirection*maxTurn_Low);
        } else {
            setValues(0, searchDirection*maxTurn_High);
        }

    }

}
