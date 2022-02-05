package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonSRXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.utility.MotorControllerFactory;

public class Climber extends SubsystemBase {

    private TalonSRX climberMotor;
    private DoubleSolenoid climberSolenoid;

    public Climber() {

//        climberMotor = MotorControllerFactory.makeTalonSRX(Constants.Climber.climberPort);
//        climberSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, Constants.Climber.solenoidPorts[0], Constants.Climber.solenoidPorts[1]);
    }
}
