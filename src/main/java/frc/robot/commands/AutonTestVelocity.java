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
        mInitTime = System.currentTimeMillis();

        mDrivetrain.resetEncoders();
        mDrivetrain.resetYaw();

        PIDConfig.setPIDF(mLeftPIDController, mRightPIDController, 0.001, 0.001, 0.001, 0.001);

    }
    @Override
    public void execute(){
        int time = getTimeElapsed();

        if (!isFinished()){
//            System.out.println("Left Trajectory Velocity: "+mLeftTrajectory.getPoints().get(time).getVelocity());
//            System.out.println("Right Trajectory Velocity: "+mRightTrajectory.getPoints().get(time).getVelocity());

            mLeftPIDController.setReference(mLeftTrajectory.getPoints().get(time).getVelocity() * conversion, CANSparkMax.ControlType.kVelocity);
            mRightPIDController.setReference(mRightTrajectory.getPoints().get(time).getVelocity() * conversion, CANSparkMax.ControlType.kVelocity);

//            System.out.println("Error: "+ (mLeftEncoder.getVelocity() - mLeftTrajectory.getPoints().get(time).getVelocity()));


//            mLeftPIDController.setReference(0.5, CANSparkMax.ControlType.kVelocity);
//            mRightPIDController.setReference(0.5, CANSparkMax.ControlType.kVelocity);

//            mDrivetrain.printRPM();
//            mDrivetrain.printPosition();

        }

    }

    @Override
    public void end(boolean interrupted){
        mDrivetrain.getLeftLeader().set(0);
        mDrivetrain.getRightLeader().set(0);
        mDrivetrain.resetEncoders();
        mDrivetrain.resetYaw();
    }

    @Override
    public boolean isFinished(){
        boolean stop = getTimeElapsed() >= mLeftTrajectory.getPoints().size() * mLeftTrajectory.getPoints().get(0).getDt()
                || getTimeElapsed() >= mRightTrajectory.getPoints().size() * mRightTrajectory.getPoints().get(0).getDt();
        return stop;
    }
}
