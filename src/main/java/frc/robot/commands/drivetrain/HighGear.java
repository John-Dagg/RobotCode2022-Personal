package frc.robot.commands.drivetrain;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class HighGear extends CommandBase {

    private Drivetrain mDrivetrain;

    public HighGear(Drivetrain subsystem){
        mDrivetrain = subsystem;
        addRequirements(mDrivetrain);
    }

    @Override
    public void initialize(){
        mDrivetrain.highGear();
    }

    @Override
    public boolean isFinished(){
        return true;
    }
}
