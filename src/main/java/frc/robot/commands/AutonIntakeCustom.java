package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;

public class AutonIntakeCustom extends CommandBase {

    private Intake mIntake;

    public AutonIntakeCustom(Intake subsystem){
        mIntake = subsystem;

        addRequirements(mIntake);
    }

    @Override
    public void initialize(){
        mIntake.extendIntake();
    }

    @Override
    public void execute(){
        mIntake.rollerIntake();
    }

    @Override
    public void end(boolean isFinished){
        mIntake.rollerStop();
    }


}