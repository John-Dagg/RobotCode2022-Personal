package frc.robot.autons;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.Constants;
import frc.robot.commands.*;

import static frc.robot.Constants.HoodState.HIGH;
import static frc.robot.Constants.IntakeState.IN;
import static frc.robot.Constants.LimelightVision.TurnDirection.LEFT;
import static frc.robot.Constants.LimelightVision.TurnDirection.RIGHT;
import frc.robot.autons.AutonGenerator;
import frc.robot.limelightvision.LimelightAlignCommand;
import frc.robot.limelightvision.VPLimelight;
import frc.robot.subsystems.*;

import java.util.ArrayList;

public class AutonRoutine {

    //Auton

//    private String[] mFiveBallAuton = {"FiveBall1", "FiveBall2", "FiveBall3"};
//    private String[] mTest = {"test"};
//    private String[] mCurve = {"Curve"};
//    private String[] mTurn = {"Turn"};
//    private String[] mThreeBall = {"ThreeBall1", "ThreeBall2"};


    public enum Routine {
        FIVE_BALL_TEST(0, new String[]{"FiveBall1", "FiveBall2", "FiveBall3"}),
        THREE_BALL_TEST(1, new String[]{"ThreeBall1", "ThreeBall2"});

        Routine(int index, String[] trajectory){
            this.index = index;
            this.trajectory = trajectory;

        }
        private int index;
        private String[] trajectory;
        private SequentialCommandGroup commands;

        public String[] getRoutineTrajectory(){
            return trajectory;
        }

        public void setCommands(SequentialCommandGroup commands) {this.commands = commands;}

        public SequentialCommandGroup getCommands() {
            return commands;
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


        switch (mRoutine) {
            case FIVE_BALL_TEST:
                    mRoutine.setCommands(new SequentialCommandGroup(
                    new InstantCommand(mIntake::extendIntake), new ShootLow(mShooter, mIndexer, Constants.DriveTrain.DriveState.AUTO_DRIVE),
                    new ParallelRaceGroup(ramseteCommands.get(0), new IntakeCargo(mIntake)),
                    new ParallelCommandGroup(new LimelightAlignCommand(mDrivetrain, mLimelightVision, Constants.LimelightVision.TurnDirection.LEFT, Constants.LimelightVision.TurnMode.AUTON),
                            new ShootClose(mShooter, mIndexer, 4, -0.72, false)),
                    new ParallelRaceGroup(ramseteCommands.get(1).andThen(ramseteCommands.get(2)), new IntakeCargo(mIntake)),
                    new ParallelCommandGroup(new LimelightAlignCommand(mDrivetrain, mLimelightVision, Constants.LimelightVision.TurnDirection.RIGHT, Constants.LimelightVision.TurnMode.AUTON),
                            new ShootClose(mShooter, mIndexer, 4, -0.72, false)),
                    new InstantCommand(mDrivetrain::stopDrive)));
                break;
            case THREE_BALL_TEST:
                mRoutine.setCommands(new SequentialCommandGroup(
                        new AutonDrive(mIntake, ramseteCommands, 0),
                        new AutonShoot(mDrivetrain, mIntake, mIndexer, mShooter, mLimelightVision, RIGHT, HIGH, IN, -0.72, 4),
                        new AutonDrive(mIntake, ramseteCommands, 1),
                        new AutonShoot(mDrivetrain, mIntake, mIndexer, mShooter, mLimelightVision, LEFT, HIGH, IN, -0.72, 4)));
                break;
            default:
                break;
        }
    }

    public SequentialCommandGroup getRoutine() {
        return mRoutine.getCommands();
    }

    public void routineInitialize() {
        mDrivetrain.resetOdometry(autonGenerator.getTrajectory(mRoutine.getRoutineTrajectory()[0]).getInitialPose());
    }
}
