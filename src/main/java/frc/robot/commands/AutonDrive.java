package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.subsystems.Intake;

import java.util.ArrayList;

public class AutonDrive extends ParallelRaceGroup {

    public AutonDrive(Intake mIntake, ArrayList<RamseteCommand> mRamseteCommands, int index) {
        super(new IntakeCargo(mIntake), mRamseteCommands.get(index));
    }

    public AutonDrive(ArrayList<RamseteCommand> mRamseteCommands, int index){
        super(mRamseteCommands.get(index));
    }
}