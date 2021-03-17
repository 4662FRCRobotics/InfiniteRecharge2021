/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.I2C;
import frc.robot.Constants;
import frc.robot.Constants.ContestantConstants;

import com.revrobotics.ColorSensorV3;
import com.revrobotics.ColorMatchResult;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.ColorMatch;

public class WheelOfFortuneRotator extends SubsystemBase {
  /**
   * Creates a new WheelOfFortuneRotator.
   */

  private final I2C.Port i2cPort = I2C.Port.kOnboard;

  private ColorSensorV3 m_colorSensor;

  private final ColorMatch m_colorMatcher;

  private String m_gameData;
  private int m_colorChangeCount;

  private final Color kBLUE;
  private final Color kGREEN;
  private final Color kRED;
  private final Color kYELLOW;

  public String m_targetColorString;
  public Color m_targetColor;

  private String m_currentColor;
  private String m_previousColor;
  private double m_colorConfidence;

  private WPI_TalonSRX m_colorWheelMotor;

  private boolean hasSensorEverReset = false;

  public WheelOfFortuneRotator() {
    instantiateColorSensor();

    m_colorMatcher = new ColorMatch();
    m_gameData = "";
    m_colorChangeCount = 0;
    m_previousColor = "";

    m_colorWheelMotor = new WPI_TalonSRX(ContestantConstants.kMOTOR_ID);
    m_colorWheelMotor.configFactoryDefault();

    //m_contestantMotor = new TalonSRX(ContestantConstants.CONTESTANT_MOTOR);

    kBLUE = ColorMatch.makeColor(ContestantConstants.Color.BLUE.getRed(), ContestantConstants.Color.BLUE.getGreen(), ContestantConstants.Color.BLUE.getBlue());
    kGREEN = ColorMatch.makeColor(ContestantConstants.Color.GREEN.getRed(), ContestantConstants.Color.GREEN.getGreen(), ContestantConstants.Color.GREEN.getBlue());
    kRED = ColorMatch.makeColor(ContestantConstants.Color.RED.getRed(), ContestantConstants.Color.RED.getGreen(), ContestantConstants.Color.RED.getBlue());
    kYELLOW = ColorMatch.makeColor(ContestantConstants.Color.YELLOW.getRed(), ContestantConstants.Color.YELLOW.getGreen(), ContestantConstants.Color.YELLOW.getBlue());

    m_colorMatcher.addColorMatch(kBLUE);
    m_colorMatcher.addColorMatch(kGREEN);
    m_colorMatcher.addColorMatch(kRED);
    m_colorMatcher.addColorMatch(kYELLOW);
  }

  @Override
  public void periodic() { // Todo: Save relevant data to member class variables and offload logic to various methods

    Color detectedColor;
    detectedColor = readSensor();

    String colorString;
    ColorMatchResult match = m_colorMatcher.matchClosestColor(detectedColor);

    if (match.color == kBLUE) {
      colorString = ContestantConstants.Color.BLUE.getName();
    } else if (match.color == kRED) {
      colorString = ContestantConstants.Color.RED.getName();
    } else if (match.color == kGREEN) {
      colorString = ContestantConstants.Color.GREEN.getName();
    } else if (match.color == kYELLOW) {
      colorString = ContestantConstants.Color.YELLOW.getName();
    } else {
      colorString = "Unknown";
    }

    m_currentColor = colorString;
    m_targetColorString = offsetColor(m_gameData);

    switch (m_targetColorString) {
      case ContestantConstants.kBLUE_STRING:
        m_targetColor = kBLUE;
        break;

      case ContestantConstants.kRED_STRING:
        m_targetColor = kRED;
        break;

      case ContestantConstants.kGREEN_STRING:
        m_targetColor = kGREEN;
        break;

      case ContestantConstants.kYELLOW_STRING:
        m_targetColor = kYELLOW;
        break;

      default: 
        m_targetColor = kRED;
        break;
    }

    m_colorConfidence = match.confidence;
    
    /*
    if (m_colorSensor.hasReset()){
      hasSensorEverReset = true;
      instantiateColorSensor();
    }
    */

    SmartDashboard.putBoolean("Has Sensor Ever Reset", hasSensorEverReset);
    SmartDashboard.putNumber("Red", detectedColor.red);
    SmartDashboard.putNumber("Green", detectedColor.green);
    SmartDashboard.putNumber("Blue", detectedColor.blue);
    SmartDashboard.putNumber("Confidence", match.confidence);
    SmartDashboard.putNumber("Color Change Count", m_colorChangeCount);
    SmartDashboard.putString("Detected Color", colorString);
    SmartDashboard.putString("Target Color", m_targetColorString);
  }

  private void instantiateColorSensor(){
    m_colorSensor = new ColorSensorV3(i2cPort);
  }

  public void detectColorChange(){
    if ((!m_previousColor.equals (m_currentColor)) && (m_colorConfidence > Constants.ContestantConstants.kCOLOR_CONFIDENCE_THRESHOLD)){
      m_colorChangeCount++;
      m_previousColor = m_currentColor;
    }
  }  

  public boolean limitReached(){
    return m_colorChangeCount >= Constants.ContestantConstants.kCHANGES_PER_ROT;
  }

  private String offsetColor(String readColor){
    String returnValue = ContestantConstants.kRED_STRING;
    switch (readColor){
      case ContestantConstants.kBLUE_STRING:
        returnValue = ContestantConstants.kRED_STRING;
        break;

      case ContestantConstants.kGREEN_STRING:
        returnValue = ContestantConstants.kYELLOW_STRING;
        break;

      case ContestantConstants.kRED_STRING:
        returnValue = ContestantConstants.kBLUE_STRING;
        break;

      case ContestantConstants.kYELLOW_STRING:
        returnValue = ContestantConstants.kGREEN_STRING;
        break;
    }
    return returnValue;
  }

  public boolean matchColor(){
    boolean returnValue = false;

    Color detectedColor;
    detectedColor = readSensor();

    ColorMatchResult match = m_colorMatcher.matchClosestColor(detectedColor);

    if ((match.color == m_targetColor) && (m_colorConfidence > Constants.ContestantConstants.kCOLOR_CONFIDENCE_THRESHOLD)) {
      returnValue = true;
    }

    return returnValue;
  }

  private Color readSensor(){

    Color detectedColor = m_colorSensor.getColor();

    double IR = m_colorSensor.getIR();

    SmartDashboard.putNumber("Red", detectedColor.red);
    SmartDashboard.putNumber("Green", detectedColor.green);
    SmartDashboard.putNumber("Blue", detectedColor.blue);
    SmartDashboard.putNumber("IR", IR);

    int proximity = m_colorSensor.getProximity();

    SmartDashboard.putNumber("Proximity", proximity);
    
    return detectedColor;
  }

  public void getGameData(){
    m_gameData = DriverStation.getInstance().getGameSpecificMessage();
    SmartDashboard.putString("Game data", m_gameData);
  }

  public void zeroColorChangeCount(){
    m_colorChangeCount = 0;
  }
 
  public void setColorWheelMotor(double speed){
    m_colorWheelMotor.set(speed);
  }
}
