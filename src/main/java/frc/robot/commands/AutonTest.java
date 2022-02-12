package frc.robot.commands;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
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
    private Trajectory mLeftTrajectory;
    private Trajectory mRightTrajectory;

    private File mLeftFile;
    private File mRightFile;

    private SparkMaxPIDController mLeftPIDController;
    private SparkMaxPIDController mRightPIDController;

    private double mInitTime;
    private boolean mStopBool;

    private String mTrajectoryDirectory = "/TrajectoryFiles/";//"/frc/robot/TrajectoryFollowing";//System.getProperty("user.dir")+"../TrajectoryFiles/"; //"java/frc/robot/TrajectoryFiles/"

    public AutonTest(Drivetrain subsystem){
        mDrivetrain = subsystem;
        addRequirements(mDrivetrain);

//        Path p1 = Paths.get(System.getProperty("user.dir"), "Documents", "GitHub", "RobotCode2022-Personal", "src", "main", "java", "frc", "robot", "TrajectoryFiles", "LeftTrajectory.csv");
//
//        Path p2 = Paths.get(System.getProperty("user.dir"), "Documents", "GitHub", "RobotCode2022-Personal", "src", "main", "java", "frc", "robot", "TrajectoryFiles", "RightTrajectory.csv");


//        File curDir = new File(System.getProperty("user.dir"));
//
//        File[] filesList = curDir.listFiles();
//
//        for(File f:filesList) {
//            System.out.println("FILE?: "+f.getName());
//        }

        System.out.println();
        mLeftFile = new File(mTrajectoryDirectory+"LeftTrajectory.csv");
        mRightFile = new File(mTrajectoryDirectory+"RightTrajectory.csv");

        mLeftTrajectory = new Trajectory(mLeftFile, REV);
        mRightTrajectory = new Trajectory(mRightFile, REV);

        mLeftPIDController = mDrivetrain.getLeftLeader().getPIDController();
        mRightPIDController = mDrivetrain.getRightLeader().getPIDController();

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

        while (!isFinished()){
//            System.out.println("Working");

            mLeftPIDController.setReference(mLeftTrajectory.getPoints().get(time).getVelocity(), CANSparkMax.ControlType.kVelocity);
            mRightPIDController.setReference(mRightTrajectory.getPoints().get(time).getVelocity(), CANSparkMax.ControlType.kVelocity);
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
