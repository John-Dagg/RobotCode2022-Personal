package frc.robot.commands;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.TrajectoryFollowing.Trajectory;
import frc.robot.subsystems.Drivetrain;
import frc.robot.utility.PIDConfig;

import java.io.File;

import static frc.robot.TrajectoryFollowing.Trajectory.TargetMotorController.REV;

public class AutonTestPosition extends CommandBase {

    private Drivetrain mDrivetrain;
    private File mLeftFile, mRightFile;
    private Trajectory mLeftTrajectory, mRightTrajectory;
    private CANSparkMax mLeftLeader, mRightLeader;
    private SparkMaxPIDController mLeftPIDController, mRightPIDController;
    private RelativeEncoder mLeftEncoder, mRightEncoder;

    private String mTrajectoryDirectory = Filesystem.getDeployDirectory().toString();

    private double mInitTime;
    private int time;

    public AutonTestPosition(Drivetrain subsystem){
        mDrivetrain = subsystem;
        addRequirements(mDrivetrain);

        mLeftFile = new File(mTrajectoryDirectory+"LeftTrajectory.csv");
        mRightFile = new File(mTrajectoryDirectory+"RightTrajectory.csv");

        mLeftTrajectory = new Trajectory(mLeftFile, REV, 1);
        mRightTrajectory = new Trajectory(mRightFile, REV, 1);

        mLeftLeader = mDrivetrain.getLeftLeader();
        mRightLeader = mDrivetrain.getRightLeader();

        mLeftPIDController = mDrivetrain.getLeftLeader().getPIDController();
        mRightPIDController = mDrivetrain.getRightLeader().getPIDController();

        mLeftEncoder = mDrivetrain.getLeftEncoder();
        mRightEncoder = mDrivetrain.getRightEncoder();

        mLeftPIDController.setFeedbackDevice(mDrivetrain.getLeftEncoder());
        mRightPIDController.setFeedbackDevice(mDrivetrain.getRightEncoder());
    }

    public double getTimeElapsed(){
        return System.currentTimeMillis() - mInitTime;
    }

    @Override
    public void initialize(){
        mInitTime = System.currentTimeMillis();
        mDrivetrain.resetYaw();
        mDrivetrain.resetEncoders();

        PIDConfig.setPID(mLeftPIDController, mRightPIDController, 0.01, 0.01, 0.01);
    }

    @Override
    public void execute(){
        if (!isFinished()){
            time = (int)getTimeElapsed();

            mLeftPIDController.setReference(mLeftTrajectory.getPoints().get(time).getPosition(), CANSparkMax.ControlType.kPosition);
            mRightPIDController.setReference(mRightTrajectory.getPoints().get(time).getPosition(), CANSparkMax.ControlType.kPosition);
        }
    }

    @Override
    public void end(boolean isFinished){
        if(isFinished){
            mLeftLeader.set(0);
            mRightLeader.set(0);
            mDrivetrain.resetEncoders();
            mDrivetrain.resetYaw();
        }
    }

    @Override
    public boolean isFinished(){
        boolean stop = getTimeElapsed() < mLeftTrajectory.getPoints().size() || getTimeElapsed() < mRightTrajectory.getPoints().size();
        return stop;
    }
}
