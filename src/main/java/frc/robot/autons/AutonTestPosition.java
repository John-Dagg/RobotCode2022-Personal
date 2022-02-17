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

public class AutonTestPosition extends CommandBase {

    //Needs a lot of work
    //Currently trying to work on Velocity Control instead

    private Drivetrain mDrivetrain;
    private File mLeftFile, mRightFile;
    private Trajectory mLeftTrajectory, mRightTrajectory;
    private CANSparkMax mLeftLeader, mRightLeader;
    private SparkMaxPIDController mLeftPIDController, mRightPIDController;
    private RelativeEncoder mLeftEncoder, mRightEncoder;

    private String mTrajectoryDirectory = Filesystem.getDeployDirectory().toString()+"/";

    private double mInitTime;
    private int time;

    public AutonTestPosition(Drivetrain subsystem){
        mDrivetrain = subsystem;
        addRequirements(mDrivetrain);

        mLeftFile = new File(mTrajectoryDirectory+"LeftTrajectory.csv");
        mRightFile = new File(mTrajectoryDirectory+"RightTrajectory.csv");

        mLeftTrajectory = new Trajectory(mLeftFile, REV, 1);
        mRightTrajectory = new Trajectory(mRightFile, REV, 1);

        //Provides access to the drivetrain motors
        mLeftLeader = mDrivetrain.getLeftLeader();
        mRightLeader = mDrivetrain.getRightLeader();

        //Allows use of the Sparkmaxes as PID controllers
        mLeftPIDController = mDrivetrain.getLeftLeader().getPIDController();
        mRightPIDController = mDrivetrain.getRightLeader().getPIDController();

        //Provides access to the drivetrain encoders
        mLeftEncoder = mDrivetrain.getLeftEncoder();
        mRightEncoder = mDrivetrain.getRightEncoder();

        mLeftPIDController.setFeedbackDevice(mDrivetrain.getLeftEncoder());
        mRightPIDController.setFeedbackDevice(mDrivetrain.getRightEncoder());
    }

    public int getTimeElapsed(){
        double timeElapsed = (System.currentTimeMillis() - mInitTime);
        return (int)timeElapsed;
    }

    @Override
    public void initialize(){
        mInitTime = System.currentTimeMillis();
        mDrivetrain.resetYaw();
        mDrivetrain.resetEncoders();

        PIDConfig.setPID(mLeftPIDController, mRightPIDController, 0.01, 0.01, 0.01);

    }


    //A big mess that I need to look at another time
    @Override
    public void execute(){
        time = getTimeElapsed();
        if (!isFinished()){

//            System.out.println("Working");
//            mLeftPIDController.setReference(mLeftTrajectory.getPoints().get(time).getPosition(), CANSparkMax.ControlType.kPosition);
//            mRightPIDController.setReference(mRightTrajectory.getPoints().get(time).getPosition(), CANSparkMax.ControlType.kPosition);

//            mLeftPIDController.setReference(-0.01, CANSparkMax.ControlType.kPosition);
//            mRightPIDController.setReference(-0.01, CANSparkMax.ControlType.kPosition);

            if (getTimeElapsed() < 1000){
                mLeftPIDController.setReference(-10, CANSparkMax.ControlType.kPosition);
                mRightPIDController.setReference(-10, CANSparkMax.ControlType.kPosition);
            } else {
                /*
                mLeftPIDController.setReference(0, CANSparkMax.ControlType.kPosition);
                mRightPIDController.setReference(0, CANSparkMax.ControlType.kPosition);

                 */
            }




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
        boolean stop = getTimeElapsed() >= mLeftTrajectory.getPoints().size() || getTimeElapsed() >= mRightTrajectory.getPoints().size();
        boolean stop2 = getTimeElapsed() > 1000;
        return stop2;
    }
}
