package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Climber;

public class ClimberControl extends CommandBase {

    Climber mClimber;

    public ClimberControl(Climber subsystem){
        mClimber = subsystem;
        addRequirements(mClimber);
    }

    @Override
    public void initialize(){
//        mClimber.disengageBrake();
    }

    @Override
    public void execute(){
        mClimber.winchRawControl();
    }

    @Override
    public void end(boolean isFinished){
        mClimber.winchStop();
//        mClimber.engageBrake();
    }




}