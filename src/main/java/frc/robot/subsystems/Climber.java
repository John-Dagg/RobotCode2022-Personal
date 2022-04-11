package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.io.Axis;
import frc.robot.utility.ControllerFactory;

import static frc.robot.Constants.Climber.*;
import static frc.robot.Constants.operatorController;

public class Climber extends SubsystemBase {


    //Motors subject to change
    private TalonSRX climberLeader, climberFollower;
    private DoubleSolenoid climberSolenoid;
//    private DoubleSolenoid brake;

    private double winchVel;

    public Climber() {

        //Create motor objects
        climberLeader = ControllerFactory.makeTalonSRX(Constants.Climber.climberPortA);
        climberFollower = ControllerFactory.makeTalonSRX(Constants.Climber.climberPortB);

        //Create solenoid objects
        climberSolenoid = ControllerFactory.makeDoubleSolenoid(solenoidPorts[1], solenoidPorts[2]);
//        brake = ControllerFactory.makeDoubleSolenoid(brakePorts[1], brakePorts[2]);

        climberFollower.follow(climberLeader);
        climberLeader.setNeutralMode(NeutralMode.Brake); //So the winch doesn't uncoil when the motor isn't being powered
        climberFollower.setNeutralMode(NeutralMode.Brake);
    }

    public void winchRawControl(){
        winchVel = deadband(operatorController.getRawAxis(Axis.LEFT_Y.getID()));

        climberLeader.set(TalonSRXControlMode.PercentOutput, winchVel);
    }

    //
    public double deadband(double value){
        //Ternary operator that takes a condition. If true returns the first and if false returns the second value
        //If the input value is greater than the deadband than return the value otherwise return 0
        return Math.abs(value) > deadband ? value : 0;
    }

    public void angleA(){
        if (climberSolenoid.get() != DoubleSolenoid.Value.kForward) climberSolenoid.set(DoubleSolenoid.Value.kForward);
    }

    public void angleB(){
        if (climberSolenoid.get() != DoubleSolenoid.Value.kReverse) climberSolenoid.set(DoubleSolenoid.Value.kReverse);
    }

    //Toggles the state of the climber solenoid
    //If the state is not equal to kForward then set it to forward
    //If the state is not equal to kReverse then set it to reverse
    public void angleClimber(){
        if (climberSolenoid.get() == DoubleSolenoid.Value.kOff) climberSolenoid.set(DoubleSolenoid.Value.kForward);
        if (climberSolenoid.get() != DoubleSolenoid.Value.kForward) climberSolenoid.set(DoubleSolenoid.Value.kForward);
        if (climberSolenoid.get() != DoubleSolenoid.Value.kReverse) climberSolenoid.set(DoubleSolenoid.Value.kReverse);
    }

    public void winchStop(){
        climberLeader.set(ControlMode.PercentOutput, 0.0);
    }

}
