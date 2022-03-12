package frc.robot.autons;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class HardCodeAuton extends CommandBase {

    private Drivetrain mDrivetrain;
    private Intake mIntake;
    private Indexer mIndexer;
    private Shooter mShooter;

    private double speed;

    private double start, currentTime, elapseTime;
    private double throttleValue, turnValue;
    private boolean stopFlag;

    public HardCodeAuton(Drivetrain subsystemA, Intake subsystemB, Indexer subsystemC, Shooter subsystemD){

        mDrivetrain = subsystemA; mIntake = subsystemB; mIndexer = subsystemC; mShooter = subsystemD;
        addRequirements(mDrivetrain, mIntake, mIndexer, mShooter);
    }

    @Override
    public void initialize(){
        System.out.println("Starting Auton");
        start = System.currentTimeMillis();
        stopFlag = false;
        mDrivetrain.mState = Constants.DriveTrain.DriveState.AUTO_DRIVE;
        mDrivetrain.resetEncoders();

        speed = 0.75;
    }

    @Override
    public void execute(){
        currentTime = System.currentTimeMillis();
        elapseTime = (currentTime - start) / 1000; //Seconds
        System.out.println("Elapsed Time: " + elapseTime);

        System.out.println("Left Wheels Distance: " + mDrivetrain.leftWheelsPosition());
        System.out.println("Right Wheels Distance: " + mDrivetrain.rightWheelsPosition());

        if (mDrivetrain.leftWheelsPosition() < 0.8 && mDrivetrain.rightWheelsPosition() < 0.8){
            System.out.println("Running phase 1");
            mDrivetrain.autonDrive(0.4, 0);
            mIntake.extendIntake(); //Internal logic that only actuates the solenoid if the solenoid is in the opposite state
            if (elapseTime > 0.5){
                mIntake.rollerIntake();
            }
            mShooter.setAnglerLow();
            mShooter.setShooterVel(-speed);
        }

        if (mDrivetrain.leftWheelsPosition() >= 1.15 || mDrivetrain.rightWheelsPosition() >= 1.15 && elapseTime < 10) {
            System.out.println("Running phase 2");
            mDrivetrain.stopDrive();
            if (elapseTime < 3) {
                mIntake.rollerIntake();
            } else {
                mIntake.rollerStop();
                mIntake.retractIntake();
            }
            mShooter.setShooterVel(-speed);
            if (elapseTime > 4.5) {
                mIndexer.feedIndexer();
            }
        }
        if (elapseTime > 10){
            mIndexer.setIndexerIdle();
            mShooter.setShooterIdle();
        }




        /***
         * No sensors to determine the balls state so a time-based failsafe is necessary.
         * Also calls end() as another layer of security
         */

        if(elapseTime > 15){
            System.out.println("Stopping");
            mDrivetrain.stopDrive();
            mIntake.retractIntake();
            mIntake.rollerStop();
            mShooter.setShooterIdle();
            mIndexer.setIndexerIdle();
            end(true);
        }
    }

    @Override
    public void end(boolean isFinished){
        mDrivetrain.stopDrive();
        mIndexer.setIndexerIdle();
        mShooter.setShooterIdle();
        mDrivetrain.mState = Constants.DriveTrain.DriveState.TELE_DRIVE_INTAKE;
        System.out.println("Ending Auton");
    }

    @Override
    public boolean isFinished(){
        if (elapseTime > 15){
            stopFlag = true;
        } else {
            stopFlag = false;
        }
        return stopFlag;
    }
}
