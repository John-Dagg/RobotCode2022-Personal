package frc.robot.commands;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.VPLimelight;
import frc.robot.utility.PIDConfig;

public class AlignTargetLimeLight extends CommandBase {

    private Drivetrain mDrivetrain;
    private VPLimelight mVision;

    private CANSparkMax mLeftLeader, mRightLeader;
    private SparkMaxPIDController mLeftPIDController, mRightPIDController;

    private double speed;

    public AlignTargetLimeLight(Drivetrain subsystemA, VPLimelight subsystemB){

        mDrivetrain = subsystemA;
        mVision = subsystemB;

        addRequirements(mDrivetrain, mVision);

        mLeftLeader = mDrivetrain.getLeftLeader();
        mRightLeader = mDrivetrain.getRightLeader();

        mLeftPIDController = mLeftLeader.getPIDController();
        mRightPIDController = mRightLeader.getPIDController();

        PIDConfig.setPID(mLeftPIDController, mRightPIDController, 0, 0, 0); //Tune eventually

    }

    @Override
    public void initialize(){
        speed = 0.25;
    }

    @Override
    public void execute(){
        if(mVision.getTargets() >= 1) {
            mVision.steadyArray();
            aimTarget();
        } else {
            mVision.flashArray();
//            findTarget();
        }
    }

    @Override
    public void end(boolean isFinished){

    }

    public void aimTarget(){
        if (mVision.getxOffset() > Constants.LimelightVision.acceptableAngleP) {
            mLeftLeader.set(-speed);
            mRightLeader.set(speed);
        } else if (mVision.getxOffset() < Constants.LimelightVision.acceptableAngleN) {
            mLeftLeader.set(speed);
            mRightLeader.set(-speed);
        }
    }

    public void findTarget(){
        mLeftLeader.set(-speed);
        mRightLeader.set(speed);
    }

    public void aimTargetPID() {
        if (mVision.getxOffset() > Constants.LimelightVision.acceptableAngleP) {
            mLeftPIDController.setReference(-25, CANSparkMax.ControlType.kVelocity); //Should be in rotations per minute
            mRightPIDController.setReference(25, CANSparkMax.ControlType.kVelocity);
        } else if (mVision.getxOffset() < Constants.LimelightVision.acceptableAngleN){
            mLeftPIDController.setReference(25, CANSparkMax.ControlType.kVelocity);
            mRightPIDController.setReference(-25, CANSparkMax.ControlType.kVelocity);
        }
    }

    public void findTargetPID() {
        mLeftPIDController.setReference(-25, CANSparkMax.ControlType.kVelocity); //Should be in rotations per minute
        mRightPIDController.setReference(25, CANSparkMax.ControlType.kVelocity);
    }

}


