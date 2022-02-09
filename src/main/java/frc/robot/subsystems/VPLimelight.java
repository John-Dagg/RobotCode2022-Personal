package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VPLimelight extends SubsystemBase {

    private NetworkTable mNetworkTable;

    private NetworkTableEntry vpTargets, vpxOffset, vpyOffset, vpTargetArea, vpTargetSkew;
    private double targets, xOffset, yOffset, targetArea, targetSkew;

    public VPLimelight(){
        mNetworkTable = NetworkTableInstance.getDefault().getTable("limelight");

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

        printNetworkTables();
    }

    public void printNetworkTables(){
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

    public double getTargets(){return targets;}
    public double getxOffset(){return xOffset;}
    public double getyOffset(){return yOffset;}
    public double getTargetArea(){return targetArea;}
    public double getTargetSkew(){return targetSkew;}

    public void flashArray(){
        NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(2);
    }

    public void steadyArray(){
        NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(0);

    }

}
