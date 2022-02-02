package frc.robot.utility;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;

public class MotorControllerFactory {

    public static CANSparkMax makeSparkMax(int port){
        CANSparkMax mSparkMax = new CANSparkMax(port, CANSparkMaxLowLevel.MotorType.kBrushless);
        mSparkMax.restoreFactoryDefaults();
        return mSparkMax;
    }

    public static TalonFX makeTalonFX(int port){
        TalonFX mTalonFX = new TalonFX(port);
        mTalonFX.configFactoryDefault();
        return mTalonFX;
    }

    public static TalonSRX makeTalonSRX(int port){
        TalonSRX mTalonSRX = new TalonSRX(port);
        mTalonSRX.configFactoryDefault();
        return  mTalonSRX;
    }
}
