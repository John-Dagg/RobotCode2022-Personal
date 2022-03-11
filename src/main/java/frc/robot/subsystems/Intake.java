package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.io.Axis;
import frc.robot.utility.MotorControllerFactory;

import static frc.robot.Constants.Intake.fourBarPorts;

public class Intake extends SubsystemBase {
    //Waiting for Design/Build/Electrical to test

    private CANSparkMax rollerBar, staticRoller;
    private DoubleSolenoid fourBar;
    private final double intakeSpeed = -0.8;

    public Intake(){

        rollerBar = MotorControllerFactory.makeSparkMax(Constants.Intake.intakeMotorPort);
//        staticRoller = MotorControllerFactory.makeSparkMax(Constants.Intake.intakeStaticMotorPort);
        fourBar = new DoubleSolenoid(fourBarPorts[0], PneumaticsModuleType.CTREPCM, fourBarPorts[1], fourBarPorts[2]);

    }

    public void rollerIntake(){
        rollerBar.set(intakeSpeed);
//        staticRoller.set(1);
    }

    public void rollerOuttake(){
        rollerBar.set(-intakeSpeed);
//        staticRoller.set(-1);

    }

    public void triggerRollerIntake(){
        if (Constants.driverController.getRawAxis(Axis.AxisID.RIGHT_TRIGGER.getID()) > 0.25){
            rollerBar.set(intakeSpeed);
//            System.out.println("intaking");
        } else if (Constants.driverController.getRawAxis(Axis.AxisID.LEFT_TRIGGER.getID()) > 0.25){
            rollerBar.set(-intakeSpeed);
//            System.out.println("outtaking");
        } else {
            rollerStop();
        }
    }


    public void rollerStop(){
        rollerBar.set(0);
//        staticRoller.set(0);
    }

    public void extendIntake(){
        if (fourBar.get() != DoubleSolenoid.Value.kForward) fourBar.set(DoubleSolenoid.Value.kForward);
    }

    public void retractIntake(){
        if (fourBar.get() != DoubleSolenoid.Value.kReverse) fourBar.set(DoubleSolenoid.Value.kReverse);
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
