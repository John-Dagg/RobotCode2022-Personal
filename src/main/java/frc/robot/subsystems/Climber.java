package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.*;
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
    private TalonSRX climberLeader, climberFollower;
    private DoubleSolenoid climberSolenoid;
    private DoubleSolenoid brake;

    public Climber() {

        climberLeader = MotorControllerFactory.makeTalonSRX(Constants.Climber.climberPortA);
        climberFollower = MotorControllerFactory.makeTalonSRX(Constants.Climber.climberPortB);

        //Pneumatics module subject to change
        climberSolenoid = new DoubleSolenoid(Constants.Climber.solenoidPorts[0], PneumaticsModuleType.CTREPCM, Constants.Climber.solenoidPorts[1], Constants.Climber.solenoidPorts[2]);
        brake = new DoubleSolenoid(Constants.Climber.brakePorts[0], PneumaticsModuleType.CTREPCM, Constants.Climber.brakePorts[1], Constants.Climber.brakePorts[2]);

        climberFollower.follow(climberLeader);
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
        System.out.println("BRAKE STATE (IN): "+brake.get());
        if (brake.get() != DoubleSolenoid.Value.kForward || brake.get() == DoubleSolenoid.Value.kOff) brake.set(DoubleSolenoid.Value.kForward);
        else if (brake.get() != DoubleSolenoid.Value.kReverse) brake.set(DoubleSolenoid.Value.kReverse);
        System.out.println("BRAKE STATE (OUT): "+brake.get());
    }

    public void angleA(){
        climberSolenoid.set(DoubleSolenoid.Value.kReverse);
    }

    public void angleB(){
        climberSolenoid.set(DoubleSolenoid.Value.kForward);
    }

    public void winchStop(){
        climberLeader.set(ControlMode.PercentOutput, 0.0);
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



}
