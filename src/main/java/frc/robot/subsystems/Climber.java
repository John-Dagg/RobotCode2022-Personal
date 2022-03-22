package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.io.Axis;
import frc.robot.utility.MotorControllerFactory;

public class Climber extends SubsystemBase {


    //Motors subject to change
    private TalonSRX climberLeader, climberFollower;
    private DoubleSolenoid climberSolenoid;
    private DoubleSolenoid brake;

    private double winchVel;

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

    public void winchRawControl(){
        winchVel = Constants.operatorController.getRawAxis(Axis.AxisID.LEFT_Y.getID());
        double turnDirection = (Constants.operatorController.getRawAxis(Axis.AxisID.LEFT_Y.getID()) > 0) ? 1 : -1;
        winchVel =  Math.abs(winchVel) > Constants.Climber.deadband ? Math.abs(winchVel): 0;

        double winch = winchVel * turnDirection;
        climberLeader.set(TalonSRXControlMode.PercentOutput, winch);

        if (climberLeader.getSensorCollection().isFwdLimitSwitchClosed()) Math.max(0, winchVel);

        climberLeader.set(TalonSRXControlMode.PercentOutput, winch);



    }

    public void winchUp(){
        climberLeader.set(TalonSRXControlMode.PercentOutput, -0.5); //Down
    }

    public void winchDown(){
        climberLeader.set(TalonSRXControlMode.PercentOutput, 0.5); //Up
    }

    public void brake(){
        if (brake.get() != DoubleSolenoid.Value.kReverse) {
            brake.set(DoubleSolenoid.Value.kReverse);
            System.out.println("Brake Engaged");
        } else if (brake.get() != DoubleSolenoid.Value.kForward) {
            brake.set(DoubleSolenoid.Value.kForward);
            System.out.println("Brake Disengaged");
        }
    }

    public void engageBrake(){
        brake.set(DoubleSolenoid.Value.kReverse);
    }

    public void disengageBrake(){
        brake.set(DoubleSolenoid.Value.kForward);
    }

    public void angleA(){
        if (climberSolenoid.get() != DoubleSolenoid.Value.kForward) climberSolenoid.set(DoubleSolenoid.Value.kForward);
    }

    public void angleB(){
        if (climberSolenoid.get() != DoubleSolenoid.Value.kReverse) climberSolenoid.set(DoubleSolenoid.Value.kReverse);
    }

    public void angleClimber(){
        if (climberSolenoid.get() == DoubleSolenoid.Value.kOff) climberSolenoid.set(DoubleSolenoid.Value.kForward);
        if (climberSolenoid.get() != DoubleSolenoid.Value.kForward) climberSolenoid.set(DoubleSolenoid.Value.kForward);
        if (climberSolenoid.get() != DoubleSolenoid.Value.kReverse) climberSolenoid.set(DoubleSolenoid.Value.kReverse);
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
