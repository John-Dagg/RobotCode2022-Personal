package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Intake;

import java.util.ArrayList;

public class AutonDrive extends ParallelRaceGroup {

    public AutonDrive(Intake mIntake, ArrayList<RamseteCommand> mRamseteCommands, int index, double wait) {
        super(new IntakeCargo(mIntake), mRamseteCommands.get(index).andThen(new WaitCommand(wait)));
    }

    public AutonDrive(ArrayList<RamseteCommand> mRamseteCommands, int index){
        super(mRamseteCommands.get(index));
    }

    public AutonDrive(Intake mIntake, ArrayList<RamseteCommand> mRamseteCommands, int index, boolean retractIntake, double wait) {
        super(new AutonIntakeCustom(mIntake), mRamseteCommands.get(index).andThen(new WaitCommand(wait)));
    }
}