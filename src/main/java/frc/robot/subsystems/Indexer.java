package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.io.Axis;
import frc.robot.utility.ControllerFactory;

public class Indexer extends SubsystemBase {

    private TalonSRX indexerMotor;

    public Indexer() {
        indexerMotor = ControllerFactory.makeTalonSRX(Constants.Indexer.indexerPort);
    }

    public void setIndexerIdle(){
        indexerMotor.set(TalonSRXControlMode.PercentOutput, 0);
    }

    public void feedIndexer(){
        indexerMotor.set(TalonSRXControlMode.PercentOutput, -1);
    }

    //For the purpose of testing if the motor works
    public void indexerTest() {
        indexerMotor.set(TalonSRXControlMode.PercentOutput, Constants.operatorController.getRawAxis(Axis.RIGHT_Y.getID()));
    }

}
