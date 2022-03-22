package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Intake;

public class IntakeCargo extends CommandBase {

    private Intake mIntake;

    public IntakeCargo(Intake subsystem){
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
        mIntake.retractIntake();
    }


}
