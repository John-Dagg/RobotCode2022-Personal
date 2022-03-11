package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ColorMatch;
import com.revrobotics.ColorSensorV3;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.io.Axis;
import frc.robot.utility.MotorControllerFactory;

public class Indexer extends SubsystemBase {

    //Waiting on Build/Electrical for testing
    //RIP Color Sensor

    private TalonSRX indexerMotor;

    private ColorSensorV3 colorSensor;
    private ColorMatch colorMatch;
    private I2C.Port mPort;
    private Color mCargoRed, mCargoBlue;
    private Color detectedColor;

    public Indexer(){

        indexerMotor = MotorControllerFactory.makeTalonSRX(Constants.Indexer.indexerPort);

        /*
        mPort = I2C.Port.kOnboard;
        colorSensor = new ColorSensorV3(mPort);
        colorMatch = new ColorMatch();

        //Custom colors to align with the cargos
        mCargoRed = new Color(0.45, 0.35, 0.15);
        mCargoBlue = new Color(0.15, 0.35, 0.45);

        colorMatch.addColorMatch(mCargoBlue);
        colorMatch.addColorMatch(mCargoRed);
        colorMatch.setConfidenceThreshold(0.92); //Custom tested threshold. Tune once robot is completed
        */

    }
    /*
    //Returns the closest color the sensor is detecting that was added to the Color Matcher
    //If the color is not within the confidence threshold to match with either color added to the Matcher returns null
    public Color getCurrentColor(){
        return detectedColor = colorMatch.matchColor(colorSensor.getColor()).color;

    }

    //Returns whether the color sensor is seeing a specific color
    public boolean getMatch(){
        boolean matched = false;
        if(detectedColor != null) {
            if (getCurrentColor().equals(mCargoBlue) || getCurrentColor().equals(mCargoBlue)) {
                matched = true;
            }
        }
        return matched;
    }

    //Checks if a ball passes into the indexer and stops it if the color sensor sees a ball
    public boolean checkIndexer(){
        boolean indexState = false;
        if (getMatch()){
            indexerMotor.set(0);
            indexState = true;
        }
        return indexState;
    }
    */

    //Idles the indexer to push ball into the chamber. Once the color sensor sees a ball stops the indexer

    public void setIndexerIdle(){
        indexerMotor.set(TalonSRXControlMode.PercentOutput, 0);
    }

    //Feeds the balls into the shooter
    public void feedIndexer(){
        indexerMotor.set(TalonSRXControlMode.PercentOutput, -0.8);
    }

    public void indexerTest() {
        indexerMotor.set(TalonSRXControlMode.PercentOutput, Constants.operatorController.getRawAxis(Axis.AxisID.RIGHT_Y.getID()));
    }






}
