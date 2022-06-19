package frc.robot.autons;

import edu.wpi.first.wpilibj2.command.*;

import static frc.robot.Constants.HoodState.HIGH;
import static frc.robot.Constants.IntakeState.IN;
import static frc.robot.Constants.IntakeState.OUT;
import static frc.robot.Constants.LimelightVision.TurnDirection.*;

import frc.robot.limelightvision.VPLimelight;
import frc.robot.subsystems.*;

import java.util.ArrayList;

/***
 * This class houses enumerations that contain all relevant information related to each auton path including
 * index, name, and pathname from Pathweaver. The enumeration has methods to set and get commands
 */

public class AutonRoutine {

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

            //MAIN ROUTINES (UNTESTED)
            case LEFT_SIDE_TWO_BALL_DEFAULT:
                mRoutine.setCommands(new SequentialCommandGroup(
                        new AutonDrive(mIntake, ramseteCommands, 0, false, 0.5),
                        new AutonShoot(mDrivetrain, mIntake, mIndexer, mShooter, mLimelightVision, RIGHT, HIGH, OUT, 3800, 4)));
                break;
            case LEFT_SIDE_TWO_BALL_ROLL:
                mRoutine.setCommands(new SequentialCommandGroup(
                        new AutonDrive(mIntake, ramseteCommands, 0, false, 0.5),
                        new AutonShoot(mDrivetrain, mIntake, mIndexer, mShooter, mLimelightVision, RIGHT, HIGH, OUT, 3825, 3.5),
                        new AutonDrive(mIntake, ramseteCommands, 1, 0.5),
                        new AutonShoot(mDrivetrain, mIntake, mIndexer, mShooter, mLimelightVision, NONE, HIGH, IN, 2500, 3.5)));
                break;
            case LEFT_SIDE_FOUR_BALL_DEFAULT:
                mRoutine.setCommands(new SequentialCommandGroup(
                        new AutonDrive(mIntake, ramseteCommands, 0, 0.5),
                        new AutonShoot(mDrivetrain, mIntake, mIndexer, mShooter, mLimelightVision, RIGHT, HIGH, IN, 3800, 3),
                        new AutonDrive(mIntake, ramseteCommands, 1, false, 0.5),
                        new AutonDrive(mIntake, ramseteCommands, 2, false, 0.5),
                    new AutonShoot(mDrivetrain, mIntake, mIndexer, mShooter, mLimelightVision, RIGHT, HIGH, OUT, 3800, 3)));

                break;
            case RIGHT_SIDE_TWO_BALL_CLOSE:
                mRoutine.setCommands(new SequentialCommandGroup(
                        new AutonDrive(mIntake, ramseteCommands, 0, 0.5),
                        new AutonShoot(mDrivetrain, mIntake, mIndexer, mShooter, mLimelightVision, RIGHT, HIGH, OUT, 3775, 4.5)));
                break;
            case RIGHT_SIDE_FOUR_BALL_CLOSE:
                mRoutine.setCommands(new SequentialCommandGroup(
                        new AutonDrive(mIntake, ramseteCommands, 0, 0.2),
                        new AutonShoot(mDrivetrain, mIntake, mIndexer, mShooter, mLimelightVision, RIGHT, HIGH, OUT, 3825, 0.8, 2.75),
                        new AutonDrive(mIntake, ramseteCommands, 1, false, 0.2),
                        new AutonDrive(mIntake, ramseteCommands, 2, false, 0.05),
                        new AutonShoot(mDrivetrain, mIntake, mIndexer, mShooter, mLimelightVision, LEFT, HIGH, OUT, 3775, 0.8, 3.0)));
                break;
            case RIGHT_SIDE_TWO_BALL_FAR:
                mRoutine.setCommands(new SequentialCommandGroup(
                        new AutonDrive(mIntake, ramseteCommands, 0, 0.5),
                        new AutonShoot(mDrivetrain, mIntake, mIndexer, mShooter, mLimelightVision, RIGHT, HIGH, IN, 3800, 3.0)));
                break;
            case RIGHT_SIDE_THREE_BALL_FAR:
                mRoutine.setCommands(new SequentialCommandGroup(
                        new AutonDrive(mIntake, ramseteCommands, 0, 0.5),
                        new AutonShoot(mDrivetrain, mIntake, mIndexer, mShooter, mLimelightVision, RIGHT, HIGH, IN, 3700, 4.0),
                        new AutonDrive(mIntake, ramseteCommands, 1, 0.5),
                        new AutonShoot(mDrivetrain, mIntake, mIndexer, mShooter, mLimelightVision, LEFT, HIGH, IN, 3850, 4.0)));
                break;
            case RIGHT_SIDE_FOUR_BALL_FAR:
                mRoutine.setCommands(new SequentialCommandGroup(
                        new AutonDrive(mIntake, ramseteCommands, 0, 0.5),
                        new AutonShoot(mDrivetrain, mIntake, mIndexer, mShooter, mLimelightVision, RIGHT, HIGH, OUT, 3750, 1, 2.5),
                        new AutonDrive(mIntake, ramseteCommands, 1, false, 0.5),
                        new AutonDrive(mIntake, ramseteCommands, 2, false, 0.5),
                        new AutonShoot(mDrivetrain, mIntake, mIndexer, mShooter, mLimelightVision, LEFT, HIGH, IN, 3850, 1, 2.5)));
                break;
            case RIGHT_SIDE_FIVE_BALL_FAR:
                mRoutine.setCommands(new SequentialCommandGroup(
                        new AutonDrive(mIntake, ramseteCommands, 0, 0.05),
                        new AutonShoot(mDrivetrain, mIntake, mIndexer, mShooter, mLimelightVision, LEFT, HIGH, IN, 3767, 0.8, 2.6),
                        new AutonDrive(mIntake, ramseteCommands, 2, 0.05),
//                        new AutonShoot(mDrivetrain, mIntake, mIndexer, mShooter, mLimelightVision, RIGHT, HIGH, IN, 3767, 1, 2.2),
                        new AutonDrive(mIntake, ramseteCommands, 3, false, 0.15), //Boolean doesn't actually matter
//                        new AutonDrive(mIntake, ramseteCommands, 4, false, 0.15),
                        new AutonShoot(mDrivetrain, mIntake, mIndexer, mShooter, mLimelightVision, LEFT, HIGH, OUT, 3800, 1, 2.3)));
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
