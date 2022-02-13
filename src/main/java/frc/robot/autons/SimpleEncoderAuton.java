package frc.robot.autons;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class SimpleEncoderAuton extends CommandBase {

    //Incomplete. Encoder based auton to work on so that I don't go insane working on trajectory following

    private Drivetrain mDrivetrain;

    public SimpleEncoderAuton(Drivetrain subsystem){
        mDrivetrain = subsystem;
    }

    @Override
    public void initialize(){

    }

    @Override
    public void execute(){

    }

    @Override
    public void end(boolean isFinished){

    }

    @Override
    public boolean isFinished(){
        return false;
    }



}
