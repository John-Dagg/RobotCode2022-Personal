package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class SimpleEncoderAuton extends CommandBase {

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
