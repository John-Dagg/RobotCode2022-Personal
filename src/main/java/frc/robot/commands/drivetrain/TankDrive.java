package frc.robot.commands.drivetrain;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class TankDrive extends CommandBase {

    private final Drivetrain mDrivetrain;

    public TankDrive(Drivetrain subsystem){
        mDrivetrain = subsystem;
        addRequirements(subsystem);
    }

    @Override
    public void execute(){
        mDrivetrain.tankDrive();
    }

    @Override
    public boolean isFinished(){
        return false;
    }

}
