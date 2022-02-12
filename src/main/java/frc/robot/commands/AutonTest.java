package frc.robot.commands;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.TrajectoryFollowing.Trajectory;
import frc.robot.subsystems.Drivetrain;

import java.io.File;
import java.nio.file.Path;
import java.nio.file.Paths;

import static frc.robot.TrajectoryFollowing.Trajectory.TargetMotorController.REV;

public class AutonTest extends CommandBase {

    // Needs a lot of work


    private Drivetrain mDrivetrain;

    private Trajectory mLeftTrajectory, mRightTrajectory;
    private File mLeftFile, mRightFile;


    private CANSparkMax mLeftLeader, mRightLeader;
    private SparkMaxPIDController mLeftPIDController, mRightPIDController;
    private RelativeEncoder mLeftEncoder, mRightEncoder;

    private double mInitTime;
    private boolean mStopBool;

    private final double conversion = 0.1429 * 6 * Math.PI * 2;

    private String mTrajectoryDirectory = Filesystem.getDeployDirectory().toString()+"/";

    public AutonTest(Drivetrain subsystem){
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

    private int getTimeElapsed(){
        double timeElapsed = System.currentTimeMillis() - mInitTime;
        return (int)timeElapsed;
    }

    @Override
    public void initialize(){
        mInitTime = System.currentTimeMillis();

        mDrivetrain.resetEncoders();
        mDrivetrain.resetYaw();

        mLeftPIDController.setP(Constants.DriveTrain.kP);
        mLeftPIDController.setP(Constants.DriveTrain.kI);
        mLeftPIDController.setP(Constants.DriveTrain.kD);
        mRightPIDController.setP(Constants.DriveTrain.kP);
        mRightPIDController.setP(Constants.DriveTrain.kI);
        mRightPIDController.setP(Constants.DriveTrain.kD);

    }
    @Override
    public void execute(){
        int time = getTimeElapsed();

        if (!isFinished()){
//            System.out.println("Working");

            if (time >= mLeftTrajectory.getPoints().size() * mLeftTrajectory.getPoints().get(0).getDt()
                    || time >= mRightTrajectory.getPoints().size() * mRightTrajectory.getPoints().get(0).getDt())
                return;

//            System.out.println("Left Trajectory Velocity: "+mLeftTrajectory.getPoints().get(time).getVelocity());
//            System.out.println("Right Trajectory Velocity: "+mRightTrajectory.getPoints().get(time).getVelocity());

            mLeftPIDController.setReference(mLeftTrajectory.getPoints().get(time).getVelocity() , CANSparkMax.ControlType.kVelocity);
            mRightPIDController.setReference(mRightTrajectory.getPoints().get(time).getVelocity(), CANSparkMax.ControlType.kVelocity);



//            mLeftPIDController.setReference(0.5, CANSparkMax.ControlType.kVelocity);
//            mRightPIDController.setReference(0.5, CANSparkMax.ControlType.kVelocity);


            double inputLeft = mLeftEncoder.getVelocity();
            double inputRight = mRightEncoder.getVelocity();

            double inchesPerSecondLeft = (inputLeft * conversion) / 60;
            double inchesPerSecondRight = (inputRight * conversion) / 60;

//            mDrivetrain.printRPM();
            mDrivetrain.printPosition();

//            System.out.println("Average rpm: " + (inputLeft + inputRight)/60);
//            System.out.println("Average inches per second: "+(inchesPerSecondLeft + inchesPerSecondRight) / 2);


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
