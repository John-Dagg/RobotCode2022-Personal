package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ColorMatch;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.utility.MotorControllerFactory;

public class Shooter extends SubsystemBase {

    private TalonFX shooterMotorLeader, shooterMotorFollower;
    private DoubleSolenoid angler;

    private double integratedTicks = 2048;
    private double maxRPM = 6000;
    private double closeVel = 0.65;
    private double farVel = 0.85;
    private double idleVel = 0;

    public Shooter(){

//        shooterMotorLeader = MotorControllerFactory.makeTalonFX(Constants.Shooter.shooterAPort);
//        shooterMotorFollower = MotorControllerFactory.makeTalonFX(Constants.Shooter.shooterBPort);

//        angler = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, Constants.Shooter.anglerPorts[0], Constants.Shooter.anglerPorts[1]);

//        shooterMotorFollower.setInverted(true);
//        shooterMotorFollower.follow(shooterMotorLeader);
//        shooterMotorLeader.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);

    }

    public void setAnglerLow(){
        angler.set(DoubleSolenoid.Value.kReverse);
    }

    public void setAnglerHigh(){
        angler.set(DoubleSolenoid.Value.kForward);
    }

    public void setShooterClose(){
        shooterMotorLeader.set(TalonFXControlMode.PercentOutput, closeVel);
    }

    public void setShooterFar(){
        shooterMotorLeader.set(TalonFXControlMode.PercentOutput, farVel);
    }

    public void setShooterIdle(){
        shooterMotorLeader.set(TalonFXControlMode.PercentOutput, idleVel);
    }

    public double getShooterVel(){
        double ticks = shooterMotorLeader.getSelectedSensorVelocity();
        double RPM = (ticks / integratedTicks) * 10 * 60;
        double percentOutput = RPM / maxRPM;
        return percentOutput;
    }

    public double getShooterCloseVel(){
        return closeVel - 0.1;
    }

    public double getShooterFarVel(){
        return farVel - 0.1;
    }


}
