package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.ColorMatch;
import com.revrobotics.ColorSensorV3;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.utility.MotorControllerFactory;

public class Indexer extends SubsystemBase {

    private CANSparkMax indexerMotor;

    private ColorSensorV3 colorSensor;
    private ColorMatch colorMatch;
    private I2C.Port mPort;
    private Color mRed, mBlue;

    public Indexer(){

        indexerMotor = MotorControllerFactory.makeSparkMax(Constants.Shooter.indexerPort);


        mPort = I2C.Port.kOnboard;
        colorSensor = new ColorSensorV3(mPort);
        colorMatch = new ColorMatch();

        mRed = new Color(255, 0 , 0);
        mBlue = new Color(0, 0, 255);

        colorMatch.addColorMatch(mRed);
        colorMatch.addColorMatch(mBlue);

    }

    public Color getCurrentColor(){
        return colorMatch.matchClosestColor(colorSensor.getColor()).color;
    }

    public boolean getMatch(){
        boolean matched = false;
        if (getCurrentColor().equals(mBlue) || getCurrentColor().equals(mRed)){
            matched = true;
        }
        return matched;
    }

    public boolean checkIndexer(){
        boolean indexState = false;
        if (getMatch()){
            indexerMotor.set(0);
            indexState = true;
        }
        return indexState;
    }

    public void setIndexerIdle(){
        if (checkIndexer()){
            indexerMotor.set(0);
        } else {
            indexerMotor.set(0.1);
        }
    }

    public void feedIndexer(){
        indexerMotor.set(0.5);
    }
}
