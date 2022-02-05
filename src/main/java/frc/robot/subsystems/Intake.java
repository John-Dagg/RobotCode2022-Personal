package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.utility.MotorControllerFactory;

public class Intake extends SubsystemBase {

    private CANSparkMax rollerBar, staticRoller;
    private DoubleSolenoid fourBar;

    public Intake(){

//        rollerBar = MotorControllerFactory.makeSparkMax(Constants.Intake.intakeMotorPort);
//        staticRoller = MotorControllerFactory.makeSparkMax(Constants.Intake.intakeStaticMotorPort);
//        fourBar = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, Constants.Intake.fourBarPorts[0], Constants.Intake.fourBarPorts[1]);

    }

    public void rollerIntake(){
        rollerBar.set(0.5);
        staticRoller.set(0.5);
    }

    public void rollerOuttake(){
        rollerBar.set(-0.5);
        staticRoller.set(-0.5);

    }

    public void rollerStop(){
        rollerBar.set(0);
        staticRoller.set(0);
    }

    public void extendIntake(){
        fourBar.set(DoubleSolenoid.Value.kForward);
    }

    public void retractIntake(){
        fourBar.set(DoubleSolenoid.Value.kReverse);
    }

    public boolean getFourBarState(){
        boolean state = true;
        if(fourBar.get() == DoubleSolenoid.Value.kForward){
            state = true;
        } else if (fourBar.get() == DoubleSolenoid.Value.kReverse){
            state = false;
        }
        return state;
    }

}
