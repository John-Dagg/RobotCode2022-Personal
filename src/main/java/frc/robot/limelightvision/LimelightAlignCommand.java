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

//    private CANSparkMax mLeftLeader, mRightLeader;
//    private SparkMaxPIDController mLeftPIDController, mRightPIDController;
//    private MotorControllerGroup mLeftMotors, mRightMotors;

    private double speed;

    private boolean stopFlag;

    private DriveState limelightMode;

    private double searchDirection;

    public LimelightAlignCommand(Drivetrain subsystemA, VPLimelight subsystemB, TurnDirection turn, TurnMode mode){

        mDrivetrain = subsystemA;
        mVision = subsystemB;

//        addRequirements(mDrivetrain, mVision);
        addRequirements(mVision);

//        mLeftLeader = mDrivetrain.getLeftLeader();
//        mRightLeader = mDrivetrain.getRightLeader();

//        mLeftMotors = mDrivetrain.getLeftMotors();
//        mRightMotors = mDrivetrain.getRightMotors();

        searchDirection = (turn == TurnDirection.LEFT) ? -1: 1;

        limelightMode = (mode == TurnMode.TELEOP) ? DriveState.TELE_LIMELIGHT : DriveState.AUTO_LIMELIGHT;

    }


        @Override
    public void initialize(){
        mDrivetrain.mState = limelightMode;
        stopFlag = false;
        speed = 0.5;
            System.out.println("Starting Alignment");
    }

    @Override
    public void execute(){
        mVision.updateTargets();
        if(mVision.getTargets() >= 1) {
            mVision.steadyArray();
            aimTarget();
            System.out.println("Aiming at Targets");
        } else {
//            mVision.flashArray();
            mDrivetrain.printMotors();
            findTarget();
            System.out.println("Finding Targets");
        }
//        System.out.println(mVision.getyOffset());

    }

    @Override
    public void end(boolean isFinished){
        mVision.steadyArray();
        if(stopFlag){
            System.out.println("Ending Alignment");
        }
        System.out.println("ENDING");
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
//            turn = Math.signum(yaw) * 0.2;
            turn = Math.signum(yaw)*MathEqs.targetLinear(Math.abs(yaw), maxTurn, deccelAngle, deadbandAngle);
            turn = turn < 0.2 ? 0.2 : turn;
        } else {
            stopFlag = true;
            end(true);
            System.out.println("Please work");
        }
        System.out.println(turn);
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

