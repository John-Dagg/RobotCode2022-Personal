package frc.robot.limelightvision;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveTrain.DriveState;
import frc.robot.subsystems.Drivetrain;
import frc.robot.utility.MathEqs;
import frc.robot.Constants.LimelightVision.*;

import static frc.robot.Constants.LimelightVision.*;

public class LimelightAlignCommand extends CommandBase {

    private final Drivetrain mDrivetrain;
    private final VPLimelight mVision;

    private CANSparkMax mLeftLeader, mRightLeader;
    private SparkMaxPIDController mLeftPIDController, mRightPIDController;
    private MotorControllerGroup mLeftMotors, mRightMotors;

    private double speed;

    private boolean stopFlag;

    private DriveState limelightMode;

    private double searchDirection = 1;

    public LimelightAlignCommand(Drivetrain subsystemA, VPLimelight subsystemB, TurnDirection turn, TurnMode mode){

        mDrivetrain = subsystemA;
        mVision = subsystemB;

        addRequirements(mDrivetrain, mVision);

        mLeftLeader = mDrivetrain.getLeftLeader();
        mRightLeader = mDrivetrain.getRightLeader();

        mLeftMotors = mDrivetrain.getLeftMotors();
        mRightMotors = mDrivetrain.getRightMotors();

        searchDirection = (turn == TurnDirection.LEFT) ? 1: -1;

        limelightMode = (mode == TurnMode.TELEOP) ? DriveState.TELE_LIMELIGHT : DriveState.AUTO_LIMELIGHT;

    }


        @Override
    public void initialize(){
        mDrivetrain.mState = limelightMode;
        stopFlag = false;
        speed = 0.5;
    }

    @Override
    public void execute(){
        mVision.updateTargets();
        if(mVision.getTargets() >= 10) {
            mVision.steadyArray();
            aimTarget();
        } else {
//            mVision.flashArray();
            mDrivetrain.printMotors();
            findTarget();
        }
        System.out.println(mVision.getyOffset());
    }

    @Override
    public void end(boolean isFinished){
        mVision.steadyArray();
        if(stopFlag){
            System.out.println("Ending Command 1");
        }
        mDrivetrain.mState = DriveState.TELE_DRIVE;
    }

    @Override
    public boolean isFinished(){
        return stopFlag;
    }

    public void aimTarget(){
        double turn = 0;
        double yaw = mVision.getxOffset();
        if (deadbandAngle < Math.abs(yaw)) {
//            mLeftMotors.set(calcTurn());
//            mRightMotors.set(-calcTurn());
//            turn = Math.signum(yaw)*calcTurn(yaw);
            turn = Math.signum(yaw)*MathEqs.targetLinear(Math.abs(yaw), maxTurn, deccelAngle, deadbandAngle);
        } else {
            stopFlag = true;
            end(true);
            System.out.println("Please work");
        }

        mVision.setValues(0, turn);

    }

    public void findTarget(){
        mVision.setValues(0, searchDirection*maxTurn);
    }

    public double calcTurn(double x){
        return (speed * Math.abs(x)) / (deccelAngle - goalAngleP)
                + (speed * goalAngleP) / (goalAngleP - deccelAngle);
    }



}


