package frc.robot.commands.drivetrain;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;

public class LowGear extends CommandBase {

    private Drivetrain mDrivetrain;

    public LowGear(Drivetrain subsystem){
        mDrivetrain = subsystem;
        addRequirements(mDrivetrain);
    }

    @Override
    public void initialize(){
        mDrivetrain.lowGear();
    }

    @Override
    public boolean isFinished(){
        return true;
    }
}
