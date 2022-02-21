package frc.robot.limelightvision;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;

public class LimelightAlignLeftCommand extends CommandBase {

    private Drivetrain mDrivetrain;
    private VPLimelight mVision;

    private CANSparkMax mLeftLeader, mRightLeader;
    private SparkMaxPIDController mLeftPIDController, mRightPIDController;
    private MotorControllerGroup mLeftMotors, mRightMotors;

    private double speed, deccelSpeed;

    private boolean stopFlag;

    public LimelightAlignLeftCommand(Drivetrain subsystemA, VPLimelight subsystemB){

        mDrivetrain = subsystemA;
        mVision = subsystemB;

        addRequirements(mDrivetrain, mVision);

        mLeftLeader = mDrivetrain.getLeftLeader();
        mRightLeader = mDrivetrain.getRightLeader();

        mLeftMotors = mDrivetrain.getLeftMotors();
        mRightMotors = mDrivetrain.getRightMotors();
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
            mLeftMotors.set(calcTurn());
            mRightMotors.set(-calcTurn());
        } else if (mVision.getxOffset() < Constants.LimelightVision.goalAngleN) {
            mLeftMotors.set(-calcTurn());
            mRightMotors.set(calcTurn());
        } else {
            mLeftMotors.set(0);
            mRightMotors.set(0);
            stopFlag = true;
            end(stopFlag);
            System.out.println("Please work");
        }
    }

    public void findTarget(){
        mLeftMotors.set(-1);
        mRightMotors.set(1);
    }

    public double calcTurn(){
        return deccelSpeed = (speed * Math.abs(mVision.getxOffset())) / (Constants.LimelightVision.deccelAngle - Constants.LimelightVision.goalAngleP)
                + (speed * Constants.LimelightVision.goalAngleP) / (Constants.LimelightVision.goalAngleP - Constants.LimelightVision.deccelAngle);
    }

}


