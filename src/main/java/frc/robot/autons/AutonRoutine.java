package frc.robot.autons;

import edu.wpi.first.wpilibj2.command.CommandGroupBase;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.AutonDrive;
import frc.robot.commands.AutonShoot;

import static frc.robot.Constants.HoodState.HIGH;
import static frc.robot.Constants.IntakeState.IN;
import static frc.robot.Constants.LimelightVision.TurnDirection.LEFT;
import static frc.robot.Constants.LimelightVision.TurnDirection.RIGHT;
import frc.robot.autons.AutonGenerator;
import frc.robot.limelightvision.VPLimelight;
import frc.robot.subsystems.*;

import java.util.ArrayList;

public class AutonRoutine {

    //Auton

    private String[] mFiveBallAuton = {"FiveBall1", "FiveBall2", "FiveBall3"};
    private String[] mTest = {"test"};
    private String[] mCurve = {"Curve"};
    private String[] mTurn = {"Turn"};
    private String[] mThreeBall = {"ThreeBall1", "ThreeBall2"};

    public enum Routine {
        FIVE_BALL_TEST(0, new String[]{"FiveBall1", "FiveBall2", "FiveBall3"}),
        THREE_BALL_TEST(1, new String[]{"ThreeBall1", "ThreeBall2"});

        Routine(int index, String[] trajectory){
            this.index = index;
            this.trajectory = trajectory;
        }
        private int index;
        public String[] trajectory;

        public String[] getRoutineTrajectory(){
            return trajectory;
        }

        public SequentialCommandGroup tklsdja() {
            return null;
        }

    }

    private ArrayList<RamseteCommand> ramseteCommands;
    private AutonGenerator autonGenerator;

    private VPLimelight mLimelightVision;
    private Drivetrain mDrivetrain;
    private Intake mIntake;
    private Shooter mShooter;
    private Indexer mIndexer;

    private final Routine mRoutine;

    public AutonRoutine(SubsystemBase[] subsystems, Routine routine){


        mDrivetrain = (Drivetrain) subsystems[0];
        mIntake = (Intake) subsystems[1];
        mShooter = (Shooter) subsystems[2];
        mIndexer = (Indexer) subsystems[3];
        mLimelightVision = (VPLimelight) subsystems[4];
        mRoutine = routine;

        ramseteCommands = autonGenerator.getAutonCommands(mRoutine.getRoutineTrajectory(), mDrivetrain);
        mDrivetrain.resetOdometry(autonGenerator.getTrajectory(mRoutine.getRoutineTrajectory()[0]).getInitialPose());

        SequentialCommandGroup threeBallTest = new SequentialCommandGroup(
                new AutonDrive(mIntake, ramseteCommands, 0),
                new AutonShoot(mDrivetrain, mIntake, mIndexer, mShooter, mLimelightVision, RIGHT, HIGH, IN, -0.72, 4),
                new AutonDrive(mIntake, ramseteCommands, 1),
                new AutonShoot(mDrivetrain, mIntake, mIndexer, mShooter, mLimelightVision, LEFT, HIGH, IN, -0.72, 4));
        SequentialCommandGroup auton = threeBallTest;
    }
}
