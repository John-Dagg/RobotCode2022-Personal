package frc.robot.commands;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.VPLimelight;
import frc.robot.utility.PIDConfig;

public class CorrectDistanceLimelight extends CommandBase {

    private Drivetrain mDrivetrain;
    private VPLimelight mVision;

    private AlignTargetLimeLight mAlign;

    private CANSparkMax mLeftLeader, mRightLeader;
    private SparkMaxPIDController mLeftPIDController, mRightPIDController;

    private double yOffset, distance, speed, targetDistance;

    public CorrectDistanceLimelight(Drivetrain subsystemA, VPLimelight subsystemB){
        mDrivetrain = subsystemA;
        mVision = subsystemB;

        addRequirements(mDrivetrain, mVision);

        mLeftLeader = mDrivetrain.getLeftLeader();
        mRightLeader = mDrivetrain.getRightLeader();

        mLeftPIDController = mLeftLeader.getPIDController();
        mRightPIDController = mRightLeader.getPIDController();

        PIDConfig.setPID(mLeftPIDController, mRightPIDController, 0, 0, 0); //Needs Tuning

        mAlign = new AlignTargetLimeLight(mDrivetrain, mVision);
    }

    @Override
    public void initialize(){
        speed = 0.25;
        targetDistance = 120; //inches
    }

    @Override
    public void execute(){
        yOffset = mVision.getyOffset();
        if (mVision.getTargets() >= 1) {
            adjustDistance();
        } else {
//            mAlign.findTarget();
        }
    }

    public double calcDistance(){
        yOffset = mVision.getyOffset();
        distance = (Constants.LimelightVision.targetHeight - Constants.LimelightVision.cameraHeight)
                / Math.tan(Constants.LimelightVision.cameraAngle + yOffset);
        return distance; //Inches
    }

    public void adjustDistance(){
        if (calcDistance() < lowBand(targetDistance)){
            mLeftLeader.set(-speed);
            mRightLeader.set(-speed);
        } else if (calcDistance() > highBand(targetDistance)) {
            mLeftLeader.set(speed);
            mRightLeader.set(speed);
        }

    }

    public double lowBand(double targetDistance){
        return targetDistance - 6;
    }

    public double highBand(double targetDistance){
        return targetDistance + 6;
    }

}
