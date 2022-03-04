package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.TalonSRXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.utility.MotorControllerFactory;

public class Climber extends SubsystemBase {

    //Waiting on Design/Build/Electrical for testing

    //Motors subject to change
//    private TalonSRX climberLeader, climberFollower;
//    private DoubleSolenoid climberSolenoid;
//    private Solenoid brake;
/*
    public Climber() {

//        climberLeader = MotorControllerFactory.makeTalonSRX(Constants.Climber.climberPortA);
//        climberFollower = MotorControllerFactory.makeTalonSRX(Constants.Climber.climberPortB);

        //Pneumatics module subject to change
//        climberSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, Constants.Climber.solenoidPorts[0], Constants.Climber.solenoidPorts[1]);
        brake = new Solenoid(PneumaticsModuleType.CTREPCM, Constants.Climber.brakePort);

        climberLeader.follow(climberFollower);
        climberLeader.setNeutralMode(NeutralMode.Brake); //So the winch doesn't uncoil when the motor isn't being powered
        climberFollower.setNeutralMode(NeutralMode.Brake);
    }

    public void winchUp(){
        climberLeader.set(TalonSRXControlMode.PercentOutput, -1.0); //Uncertain about direction
    }

    public void winchDown(){
        climberLeader.set(TalonSRXControlMode.PercentOutput, 1.0); //Uncertain about direction
    }

    public void brake(){
        brake.set(true);
    }

    public void angleA(){
        climberSolenoid.set(DoubleSolenoid.Value.kReverse);
    }

    public void angleB(){
        climberSolenoid.set(DoubleSolenoid.Value.kForward);
    }

    public boolean getAngle(){
        if(climberSolenoid.get() == DoubleSolenoid.Value.kReverse){
            return false; // Angle A
        } else if(climberSolenoid.get() == DoubleSolenoid.Value.kForward){
            return true; // Angle B
        } else {
            return false; // Should never happen
        }
    }

 */

}
