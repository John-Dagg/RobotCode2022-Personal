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
    }

    @Override
    public void execute(){
        currentTime = System.currentTimeMillis();
        elapseTime = (currentTime - elapseTime) * 1000; //Seconds

        System.out.println("Left Wheels Distance: " + mDrivetrain.leftWheelsPosition());
        System.out.println("Right Wheels Distance: " + mDrivetrain.rightWheelsPosition());

        if (mDrivetrain.leftWheelsPosition() < 2 && mDrivetrain.rightWheelsPosition() < 2){;
            mDrivetrain.autonDrive(0.1, 0);
            mIntake.extendIntake(); //Internal logic that only actuates the solenoid if the solenoid is in the opposite state
            mIntake.rollerIntake();
            mShooter.setAnglerLow();
        }
        if (mDrivetrain.leftWheelsPosition() >= 2 || mDrivetrain.rightWheelsPosition() >= 2){
            mIntake.rollerStop();
            mIntake.retractIntake();
            mShooter.setShooterClose();
            if(mShooter.getShooterVel() > mShooter.getShooterCloseVel()){
                mIndexer.feedIndexer();
            }
            end(true);
        }
        /***
         * No sensors to determine the balls state so a time-based failsafe is necessary.
         * Also calls end() as another layer of security
         */

        if(elapseTime > 5){
            mDrivetrain.stopDrive();
            mIntake.retractIntake();
            mIntake.rollerStop();
            mShooter.setShooterIdle();
            mIndexer.setIndexerIdle();
        }
    }

    @Override
    public void end(boolean isFinished){
        mDrivetrain.stopDrive();
        mDrivetrain.mState = Constants.DriveTrain.DriveState.TELE_DRIVE;
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
