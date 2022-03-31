package frc.robot.autons;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.commands.AutonDrive;
import frc.robot.commands.*;

import static frc.robot.Constants.HoodState.HIGH;
import static frc.robot.Constants.IntakeState.IN;
import static frc.robot.Constants.IntakeState.OUT;
import static frc.robot.Constants.LimelightVision.TurnDirection.*;

import frc.robot.limelightvision.VPLimelight;
import frc.robot.subsystems.*;

import java.util.ArrayList;

public class AutonRoutine {

    //Auton

//    private String[] mFiveBallAuton = {"FiveBall1", "FiveBall2", "FiveBall3"};
//    private String[] mTest = {"test"};
//    private String[] mThreeBall = {"ThreeBall1", "ThreeBall2"};


    public enum Routine {
//        FIVE_BALL_TEST(0, "[TEST] Five Ball", new String[]{"FiveBall1", "FiveBall2", "FiveBall3"}),
//        THREE_BALL_TEST(1,"[TEST] Three Ball", new String[]{"ThreeBall1", "ThreeBall2"}),

        LEFT_SIDE_TWO_BALL_DEFAULT(1,"Left Side 2 Ball", new String[]{"LS-2BD-AB"}),
        LEFT_SIDE_TWO_BALL_ROLL(2,"Left Side 2 Ball Roll", new String[]{"LS-2BR-AB", "LS-2BR-BC"}),
        LEFT_SIDE_FOUR_BALL_DEFAULT(3,"Left Side 4 Ball", new String[]{"LS-4BD-AB", "LS-4BD-BC", "LS-4BD-CD"}),
        RIGHT_SIDE_TWO_BALL_CLOSE(4,"Close Right Side 2 Ball", new String[]{"RS-2BC-AB"}),
        RIGHT_SIDE_FOUR_BALL_CLOSE(5,"Close Right Side 4 Ball", new String[]{"RS-4BC-AB", "RS-4BC-BC", "RS-4BC-CD"}),
        RIGHT_SIDE_TWO_BALL_FAR(6,"Far Right Side 2 Ball", new String[]{"RS-2BF-AB"}),
        RIGHT_SIDE_THREE_BALL_FAR(7,"Far Right Side 3 Ball", new String[]{"RS-3BF-AB", "RS-3BF-BC"}),
        RIGHT_SIDE_FOUR_BALL_FAR(8,"Far Right Side 4 Ball", new String[]{"RS-4BF-AB", "RS-4BF-BC", "RS-4BF-CD"}),
        RIGHT_SIDE_FIVE_BALL_FAR(9,"Far Right Side 5 Ball", new String[]{"RS-5BF-AB", "RS-5BF-BC", "RS-5BF-CD", "RS-5BF-DE", "RS-5BF-EF"});

        Routine(int index, String name, String[] trajectory){
            this.index = index;
            this.name = name;
            this.trajectory = trajectory;

        }
        private final int index;
        private final String name;
        private final String[] trajectory;
        private SequentialCommandGroup commands;

        public String[] getRoutineTrajectory(){
            return trajectory;
        }

        public void setCommands(SequentialCommandGroup commands) {this.commands = commands;}

        public SequentialCommandGroup getCommands() {
            return commands;
        }

        public String getName() {return name; }

    }

    private ArrayList<RamseteCommand> ramseteCommands;
    private final AutonGenerator autonGenerator = new AutonGenerator();

    private final VPLimelight mLimelightVision;
    private final Drivetrain mDrivetrain;
    private final Intake mIntake;
    private final Shooter mShooter;
    private final Indexer mIndexer;

    private final Routine mRoutine;

    public AutonRoutine(SubsystemBase[] subsystems, Routine routine){

        mDrivetrain = (Drivetrain) subsystems[0];
        mIntake = (Intake) subsystems[1];
        mShooter = (Shooter) subsystems[2];
        mIndexer = (Indexer) subsystems[3];
        mLimelightVision = (VPLimelight) subsystems[4];
        mRoutine = routine;

        try {
            ramseteCommands = autonGenerator.getAutonCommands(mRoutine.getRoutineTrajectory(), mDrivetrain);
        }
        catch (NullPointerException e) {
            System.err.println(e + " ");
        }
        /***
         * These switch-case statements handle how each autonomous routine is structured:
         * A) THREE_BALL_TEST
         *    1) Intake 1 cargo while driving from position A to B
         *    2) Align angle via Limelight while revving shooter, shoot 2 cargo via running indexer when aligned
         *    3) Intake 1 cargo while driving from position B to C
         *    4) Align angle via Limelight while revving shooter, shoot 1 cargo via running indexer when aligned
         */

        switch (mRoutine) {
//            case FIVE_BALL_TEST:
//                    mRoutine.setCommands(new SequentialCommandGroup(
//                    new InstantCommand(mIntake::extendIntake), new ShootLow(mShooter, mIndexer, Constants.DriveTrain.DriveState.AUTO_DRIVE),
//                    new ParallelRaceGroup(ramseteCommands.get(0), new IntakeCargo(mIntake)),
//                    new ParallelCommandGroup(new LimelightAlignCommand(mDrivetrain, mLimelightVision, Constants.LimelightVision.TurnDirection.LEFT, Constants.LimelightVision.TurnMode.AUTON),
//                            new ShootClose(mShooter, mIndexer, 4, -0.68, false)),
//                    new ParallelRaceGroup(ramseteCommands.get(1).andThen(ramseteCommands.get(2)), new IntakeCargo(mIntake)),
//                    new ParallelCommandGroup(new LimelightAlignCommand(mDrivetrain, mLimelightVision, Constants.LimelightVision.TurnDirection.RIGHT, Constants.LimelightVision.TurnMode.AUTON),
//                            new ShootClose(mShooter, mIndexer, 4, -0.67, false)),
//                    new InstantCommand(mDrivetrain::stopDrive)));
//                break;
//            case THREE_BALL_TEST:
//                mRoutine.setCommands(new SequentialCommandGroup(
//                        new AutonDrive(mIntake, ramseteCommands, 0),
//                        new AutonShoot(mDrivetrain, mIntake, mIndexer, mShooter, mLimelightVision, RIGHT, HIGH, IN, -0.66, 3.5),
//                        new AutonDrive(mIntake, ramseteCommands, 1),
//                        new AutonShoot(mDrivetrain, mIntake, mIndexer, mShooter, mLimelightVision, RIGHT, HIGH, IN, -0.68, 4)));
//                        new InstantCommand(mDrivetrain::stopDrive);
//                break;

            //MAIN ROUTINES (UNTESTED)
            case LEFT_SIDE_TWO_BALL_DEFAULT:
                mRoutine.setCommands(new SequentialCommandGroup(
                        new AutonDrive(mIntake, ramseteCommands, 0),
                        new AutonShoot(mDrivetrain, mIntake, mIndexer, mShooter, mLimelightVision, RIGHT, HIGH, IN, 0.7, 4)));
                break;
            case LEFT_SIDE_TWO_BALL_ROLL:
                mRoutine.setCommands(new SequentialCommandGroup(
                        new AutonDrive(mIntake, ramseteCommands, 0),
                        new AutonShoot(mDrivetrain, mIntake, mIndexer, mShooter, mLimelightVision, RIGHT, HIGH, IN, 0.7, 3.5),
                        new AutonDrive(mIntake, ramseteCommands, 1),
                        new AutonShoot(mDrivetrain, mIntake, mIndexer, mShooter, mLimelightVision, NONE, HIGH, IN, 0.4, 3.5)));
                break;
            case LEFT_SIDE_FOUR_BALL_DEFAULT:
                mRoutine.setCommands(new SequentialCommandGroup(
                        new AutonDrive(mIntake, ramseteCommands, 0),
                        new AutonShoot(mDrivetrain, mIntake, mIndexer, mShooter, mLimelightVision, RIGHT, HIGH, IN, 0.75, 3),
                        new AutonDrive(mIntake, ramseteCommands, 1, false),
                        new AutonDrive(mIntake, ramseteCommands, 2, false),
                    new AutonShoot(mDrivetrain, mIntake, mIndexer, mShooter, mLimelightVision, RIGHT, HIGH, OUT, 0.73, 3)));

                break;
            case RIGHT_SIDE_TWO_BALL_CLOSE:
                mRoutine.setCommands(new SequentialCommandGroup(
                        new AutonDrive(mIntake, ramseteCommands, 0),
                        new AutonShoot(mDrivetrain, mIntake, mIndexer, mShooter, mLimelightVision, RIGHT, HIGH, OUT, 0.75, 4.5)));
                break;
            case RIGHT_SIDE_FOUR_BALL_CLOSE:
                mRoutine.setCommands(new SequentialCommandGroup(
                        new AutonDrive(mIntake, ramseteCommands, 0),
                        new AutonShoot(mDrivetrain, mIntake, mIndexer, mShooter, mLimelightVision, RIGHT, HIGH, IN, 0.68, 1, 2.5),
                        new AutonDrive(mIntake, ramseteCommands, 1, false),
                        new AutonDrive(mIntake, ramseteCommands, 2, false),
                        new AutonShoot(mDrivetrain, mIntake, mIndexer, mShooter, mLimelightVision, LEFT, HIGH, IN, 0.73, 1, 2.5)));
                break;
            case RIGHT_SIDE_TWO_BALL_FAR:
                mRoutine.setCommands(new SequentialCommandGroup(
                        new AutonDrive(mIntake, ramseteCommands, 0),
                        new AutonShoot(mDrivetrain, mIntake, mIndexer, mShooter, mLimelightVision, RIGHT, HIGH, IN, 0.68, 5.0)));
                break;
            case RIGHT_SIDE_THREE_BALL_FAR:
                mRoutine.setCommands(new SequentialCommandGroup(
                        new AutonDrive(mIntake, ramseteCommands, 0),
                        new AutonShoot(mDrivetrain, mIntake, mIndexer, mShooter, mLimelightVision, RIGHT, HIGH, IN, 0.68, 4.0),
                        new AutonDrive(mIntake, ramseteCommands, 1),
                        new AutonShoot(mDrivetrain, mIntake, mIndexer, mShooter, mLimelightVision, LEFT, HIGH, IN, 0.72, 4.0)));
                break;
            case RIGHT_SIDE_FOUR_BALL_FAR:
                mRoutine.setCommands(new SequentialCommandGroup(
                        new AutonDrive(mIntake, ramseteCommands, 0),
                        new AutonShoot(mDrivetrain, mIntake, mIndexer, mShooter, mLimelightVision, RIGHT, HIGH, IN, 0.68, 1, 2.5),
                        new AutonDrive(mIntake, ramseteCommands, 1, false),
                        new AutonDrive(mIntake, ramseteCommands, 2, false),
                        new AutonShoot(mDrivetrain, mIntake, mIndexer, mShooter, mLimelightVision, LEFT, HIGH, IN, 0.72, 1, 2.5)));
                break;
            case RIGHT_SIDE_FIVE_BALL_FAR:
                mRoutine.setCommands(new SequentialCommandGroup(
                        new AutonDrive(mIntake, ramseteCommands, 0),
                        new AutonShoot(mDrivetrain, mIntake, mIndexer, mShooter, mLimelightVision, LEFT, HIGH, IN, 0.68, 1, 2.2),
                        new AutonDrive(mIntake, ramseteCommands, 2),
                        new AutonShoot(mDrivetrain, mIntake, mIndexer, mShooter, mLimelightVision, RIGHT, HIGH, IN, 0.73, 1, 2),
                        new AutonDrive(mIntake, ramseteCommands, 3, false), //Boolean doesn't actually matter
                        new AutonDrive(mIntake, ramseteCommands, 4, false),
                        new AutonShoot(mDrivetrain, mIntake, mIndexer, mShooter, mLimelightVision, LEFT, HIGH, OUT, 0.73, 1, 2.3)));
                break;
            default:
                break;
        }
    }

    public SequentialCommandGroup getRoutine() {
        return mRoutine.getCommands();
    }

    public String getName() {
        return mRoutine.getName();
    }

    public void routineInitialize() {
        mDrivetrain.resetOdometry(autonGenerator.getTrajectory(mRoutine.getRoutineTrajectory()[0]).getInitialPose());
    }

}
