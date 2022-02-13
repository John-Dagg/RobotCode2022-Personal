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

public class AutonTestVelocity extends CommandBase {

    // Needs a lot of work


    private Drivetrain mDrivetrain;

    private Trajectory mLeftTrajectory, mRightTrajectory;
    private File mLeftFile, mRightFile;


    private CANSparkMax mLeftLeader, mRightLeader;
    private SparkMaxPIDController mLeftPIDController, mRightPIDController;
    private RelativeEncoder mLeftEncoder, mRightEncoder;

    private Thread mThread;


    private double mInitTime;
    private boolean mStopBool;

    //inches per second to rotations per minute
    private final double conversion = ((1.0*(6 * Math.PI)) / 7) / 60;

    private String mTrajectoryDirectory = Filesystem.getDeployDirectory().toString()+"/";

    public AutonTestVelocity(Drivetrain subsystem){
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

//        mLeftEncoder.setVelocityConversionFactor(conversion);
//        mRightEncoder.setVelocityConversionFactor(conversion);


    }

    private int getTimeElapsed(){
        double timeElapsed = System.currentTimeMillis() - mInitTime;
        return (int)timeElapsed;
    }

    @Override
    public void initialize(){

        mStopBool = false;
        mInitTime = System.currentTimeMillis();

        mDrivetrain.resetEncoders();
        mDrivetrain.resetYaw();

        PIDConfig.setPIDF(mLeftPIDController, mRightPIDController, 0.001, 0, 0, 0);

        mThread = new Thread(this::run);
        mThread.start();

    }

    public void run(){
        int time = getTimeElapsed();

        while (!isFinished() && !mStopBool){

            if (time >= mLeftTrajectory.getPoints().size() * mLeftTrajectory.getPoints().get(0).getDt()
                    || time >= mRightTrajectory.getPoints().size() * mRightTrajectory.getPoints().get(0).getDt())
                return;



            mLeftPIDController.setReference(mLeftTrajectory.getPoints().get(time).getVelocity() * conversion, CANSparkMax.ControlType.kVelocity);
            mRightPIDController.setReference(mRightTrajectory.getPoints().get(time).getVelocity() * conversion, CANSparkMax.ControlType.kVelocity);


        }

    }

    @Override
    public void end(boolean interrupted){
        mRightLeader.set(0);
        mLeftLeader.set(0);
        mDrivetrain.resetEncoders();
        mDrivetrain.resetYaw();
        mStopBool = true;
    }

    @Override
    public boolean isFinished(){
        boolean stop = getTimeElapsed() >= mLeftTrajectory.getPoints().size() * mLeftTrajectory.getPoints().get(0).getDt()
                || getTimeElapsed() >= mRightTrajectory.getPoints().size() * mRightTrajectory.getPoints().get(0).getDt();
        return stop;
    }
}
