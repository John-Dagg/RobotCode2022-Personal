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

    private double speed, deccelSpeed;

    private boolean stopFlag;

    public AlignTargetLimeLight(Drivetrain subsystemA, VPLimelight subsystemB){

        mDrivetrain = subsystemA;
        mVision = subsystemB;

        addRequirements(mDrivetrain, mVision);

        mLeftLeader = mDrivetrain.getLeftLeader();
        mRightLeader = mDrivetrain.getRightLeader();

//        mLeftPIDController = mLeftLeader.getPIDController();
//        mRightPIDController = mRightLeader.getPIDController();

//        PIDConfig.setPID(mLeftPIDController, mRightPIDController, 0, 0, 0); //Tune eventually

    }

    @Override
    public void initialize(){
        stopFlag = false;
        speed = 0.5;
    }

    @Override
    public void execute(){
        mVision.updateTargets();
        if(mVision.getTargets() >= 1) {
            mVision.steadyArray();
            aimTarget();
        } else {
//            mVision.flashArray();
            findTarget();
        }
    }

    @Override
    public void end(boolean isFinished){
        mVision.steadyArray();
        if(stopFlag){
            System.out.println("Ending Command 1");
        }
    }

    @Override
    public boolean isFinished(){
        return stopFlag;
    }

    public void aimTarget(){
        if (mVision.getxOffset() > Constants.LimelightVision.goalAngleP) {
            mLeftLeader.set(-calcTurn());
            mRightLeader.set(calcTurn());
        } else if (mVision.getxOffset() < Constants.LimelightVision.goalAngleN) {
            mLeftLeader.set(calcTurn());
            mRightLeader.set(-calcTurn());
        } else {
            mLeftLeader.set(0);
            mRightLeader.set(0);
            stopFlag = true;
            end(stopFlag);
            System.out.println("Please work");
        }
    }

    public void findTarget(){
        mLeftLeader.set(-0.5);
        mRightLeader.set(0.5);
    }

    public void aimTargetPID() {
        if (mVision.getxOffset() > Constants.LimelightVision.goalAngleP) {
            mLeftPIDController.setReference(-25, CANSparkMax.ControlType.kVelocity); //Should be in rotations per minute
            mRightPIDController.setReference(25, CANSparkMax.ControlType.kVelocity);
        } else if (mVision.getxOffset() < Constants.LimelightVision.goalAngleN){
            mLeftPIDController.setReference(25, CANSparkMax.ControlType.kVelocity);
            mRightPIDController.setReference(-25, CANSparkMax.ControlType.kVelocity);
        }
    }

    public void findTargetPID() {
        mLeftPIDController.setReference(-25, CANSparkMax.ControlType.kVelocity); //Should be in rotations per minute
        mRightPIDController.setReference(25, CANSparkMax.ControlType.kVelocity);
    }

    public double calcTurn(){
        return deccelSpeed = (speed * Math.abs(mVision.getxOffset())) / (Constants.LimelightVision.deccelAngle - Constants.LimelightVision.goalAngleP)
                + (speed * Constants.LimelightVision.goalAngleP) / (Constants.LimelightVision.goalAngleP - Constants.LimelightVision.deccelAngle);
    }

}


