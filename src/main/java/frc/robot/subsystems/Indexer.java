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
    private Color mCargoRed, mCargoBlue;
    private Color detectedColor;

    public Indexer(){

//        indexerMotor = MotorControllerFactory.makeSparkMax(Constants.Indexer.indexerPort);


        mPort = I2C.Port.kOnboard;
//        colorSensor = new ColorSensorV3(mPort);
        colorMatch = new ColorMatch();

        mCargoRed = new Color(0.45, 0.35, 0.15);
        mCargoBlue = new Color(0.15, 0.35, 0.45);

        colorMatch.addColorMatch(mCargoBlue);
        colorMatch.addColorMatch(mCargoRed);
        colorMatch.setConfidenceThreshold(0.92);

    }

    public Color getCurrentColor(){
        return detectedColor = colorMatch.matchColor(colorSensor.getColor()).color;

    }

    public boolean getMatch(){
        boolean matched = false;
        if(detectedColor != null) {
            if (getCurrentColor().equals(mCargoBlue) || getCurrentColor().equals(mCargoBlue)) {
                matched = true;
            }
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
