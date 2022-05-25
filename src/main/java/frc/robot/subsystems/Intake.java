package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.io.Axis;
import frc.robot.utility.ControllerFactory;

import static frc.robot.Constants.Intake.fourBarPorts;
import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.*;
import static frc.robot.Constants.Intake.intakeMotorPort;

public class Intake extends SubsystemBase {

    private CANSparkMax rollerBar, staticRoller;
    private DoubleSolenoid fourBar;
    private RelativeEncoder mEncoder;
    private final double intakeSpeed = 1;

    private double start, elapsedTime;
    private int x = 0;

    public Intake(){

        //Creates motor and solenoid objects
        rollerBar = ControllerFactory.makeSparkMax(intakeMotorPort);
        fourBar = ControllerFactory.makeDoubleSolenoid(fourBarPorts[1], fourBarPorts[2]);
    }

    //Unused
    public void stallFailsafe(){

        if (rollerBar.getAppliedOutput() > 0.5 && mEncoder.getVelocity() > 0) x = 0;

        //0.5 is an arbitrary value that just indicates if the motor is receiving current
        //If the motor is receiving current and the encoder isn't reading turns the motor must be stalling

        if (x < 1){
            if ((rollerBar.getAppliedOutput() > 1.4) && (mEncoder.getVelocity() < 1)) {
                start = System.currentTimeMillis();
            }
            x++;
        }
        if ((rollerBar.getAppliedOutput() > 1.4) && (mEncoder.getVelocity() < 1)){
            elapsedTime = System.currentTimeMillis() - start;
            if ((elapsedTime / 1000) > 1) rollerBar.set(0);
        }
    }

    public void rollerIntake(){
        rollerBar.set(-intakeSpeed);
    }

    public void rollerOuttake(){
        rollerBar.set(intakeSpeed);
    }

    //If a trigger is pressed enough then perform the desired action of intaking or outtaking depending on trigger
    public void triggerRollerIntake(){
        if (Constants.driverController.getRawAxis(Axis.RIGHT_TRIGGER.getID()) > 0.25){
            rollerIntake();
        } else if (Constants.driverController.getRawAxis(Axis.LEFT_TRIGGER.getID()) > 0.25){
            rollerOuttake();
        } else rollerStop();
    }


    public void rollerStop(){
        rollerBar.set(0);
    }

    public void extendIntake(){
        if (fourBar.get() != kReverse) fourBar.set(kReverse);
    }

    public void retractIntake(){
        if (fourBar.get() != kForward) fourBar.set(kForward);
    }

    public void toggleFourBar(){
        if (fourBar.get() == kOff) fourBar.set(kForward);
        if (fourBar.get() != kForward) fourBar.set(kForward);
        if (fourBar.get() != kReverse) fourBar.set(kReverse);
    }



}
