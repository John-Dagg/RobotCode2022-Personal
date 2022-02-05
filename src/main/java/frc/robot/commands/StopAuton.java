package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class StopAuton extends CommandBase {

    private Drivetrain mDrivetrain;

    public StopAuton(Drivetrain subsystem){
        mDrivetrain = subsystem;
        addRequirements(mDrivetrain);
    }

    public void initialize(){
        mDrivetrain.stopDrive();
    }

    public void execute(){
        mDrivetrain.stopDrive();
    }
}
