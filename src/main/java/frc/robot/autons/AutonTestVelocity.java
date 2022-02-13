package frc.robot.autons;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.trajectoryfollowing.Trajectory;
import frc.robot.subsystems.Drivetrain;
import frc.robot.utility.PIDConfig;

import java.io.File;

import static frc.robot.trajectoryfollowing.Trajectory.TargetMotorController.REV;

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
    private double maxVelLeft, maxVelRight, mLeftPercentOutput, mRightPercentOutput;

    //inches per second to rotations per minute
    private final double conversion = (((6.0 * Math.PI) / (7.0)) * 60.0);

    //The trajectories are added to the robot through the deploy folder
    private String mTrajectoryDirectory = Filesystem.getDeployDirectory().toString()+"/";

    public AutonTestVelocity(Drivetrain subsystem){
        mDrivetrain = subsystem;
        addRequirements(mDrivetrain);

        mLeftFile = new File(mTrajectoryDirectory+"LeftTrajectory.csv");
        mRightFile = new File(mTrajectoryDirectory+"RightTrajectory.csv");

        mLeftTrajectory = new Trajectory(mLeftFile, REV, 1);
        mRightTrajectory = new Trajectory(mRightFile, REV, 1);


        //Allows access to the drivetrain motors
        mLeftLeader = mDrivetrain.getLeftLeader();
        mRightLeader = mDrivetrain.getRightLeader();

        //Turns sparkMaxes into PID controllers
        mLeftPIDController = mDrivetrain.getLeftLeader().getPIDController();
        mRightPIDController = mDrivetrain.getRightLeader().getPIDController();

        //Returns drivetrain encoders for use in the auton
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

        //Finds the max velocity of each trajectory
        maxVelLeft = mLeftTrajectory.findMaxVelocity();
        maxVelRight = mRightTrajectory.findMaxVelocity();

        mDrivetrain.resetEncoders();
        mDrivetrain.resetYaw();

        PIDConfig.setPIDF(mLeftPIDController, mRightPIDController, 0.5, 0.01, 0.01, 0.01); // Can't be zero?

        //Should be faster than using the every 20 millisecond execute method
        mThread = new Thread(this::run);
        mThread.start();

    }

    private void run(){


        while (!isFinished() && !mStopBool){
            int time = getTimeElapsed();

            //Exits the method if the time elapsed is greater than the time the trajectory should take. Extra precaution based on 2021 Code Example
            if (time >= mLeftTrajectory.getPoints().size() * mLeftTrajectory.getPoints().get(0).getDt()
                    || time >= mRightTrajectory.getPoints().size() * mRightTrajectory.getPoints().get(0).getDt())
                break;


            //Even when setReference is zero the robot still moves at max velocity. Can't find good documentation for what setReference actually does
/*
            mLeftPIDController.setReference((-mLeftTrajectory.getPoints().get(time).getVelocity() * conversion), CANSparkMax.ControlType.kVelocity);
            mRightPIDController.setReference((-mRightTrajectory.getPoints().get(time).getVelocity() * conversion), CANSparkMax.ControlType.kVelocity);


            //Prints the velocity every 20 loops hopefully
            if (x % 20 == 0) {
                System.out.println(-mRightTrajectory.getPoints().get(time).getVelocity() * conversion);

            }
            x++;
 */
            /**
             *Workaround of the setReference method
             *Converts velocities to percents to provide to the motors. Unfortunately circumvents PID control.
             *If the motors never reach a max velocity this won't work well and the robot will move much faster than expected
            */
            mLeftPercentOutput = mLeftTrajectory.getPoints().get(time).getVelocity() / maxVelLeft;
            mRightPercentOutput = mRightTrajectory.getPoints().get(time).getVelocity() / maxVelRight;

            mLeftLeader.set(mLeftPercentOutput);
            mRightLeader.set(mRightPercentOutput);

//            System.out.println(-mLeftTrajectory.getPoints().get(time).getVelocity());


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
        //Finishes trajectory when the time elapsed is greater than the time the trajectory should run for.
        //Gets the time the trajectory should run for by multiplying the amount of trajectory points
        //by the difference in time between points
        boolean stop = getTimeElapsed() >= mLeftTrajectory.getPoints().size() * mLeftTrajectory.getPoints().get(0).getDt()
                || getTimeElapsed() >= mRightTrajectory.getPoints().size() * mRightTrajectory.getPoints().get(0).getDt();
        return stop;
    }
}
