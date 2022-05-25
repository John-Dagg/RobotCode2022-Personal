package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;

public class AutonIntakeCustom extends CommandBase {

    private Intake mIntake;
    private boolean retractIntake;

    public AutonIntakeCustom(Intake subsystem, boolean retractIntake){
        mIntake = subsystem;
        this.retractIntake = retractIntake;

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
        if (retractIntake) mIntake.retractIntake();
    }


}