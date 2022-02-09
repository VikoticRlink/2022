// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import com.revrobotics.ColorSensorV3;

public class ColorSensor extends SubsystemBase {
  /** Creates a new ColorSensor. */
  public ColorSensor() {}

  /** Creates a new LightSensor. */
  private final I2C.Port i2cPort = I2C.Port.kOnboard;
  private final ColorSensorV3 m_colorSensor = new ColorSensorV3(i2cPort);
  //Color calibration values
  private double redCorrection = -0.31;
  private double blueCorrection = -0.19;
  private double greenCorrection = -0.48;
  
  public int proximity;
  public String DetectedColor ="U";


  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    
    Color detectedColor = m_colorSensor.getColor();
    double IR = m_colorSensor.getIR();
    proximity = m_colorSensor.getProximity();

    double myRed = (detectedColor.red + redCorrection) * 255;
    double myGreen = (detectedColor.green + greenCorrection) * 255;
    double myBlue = (detectedColor.blue + blueCorrection) * 255;

    SmartDashboard.putNumber("raw Red", detectedColor.red);
    SmartDashboard.putNumber("raw Green", detectedColor.green);
    SmartDashboard.putNumber("raw Blue", detectedColor.blue);

    SmartDashboard.putNumber("Red", myRed);
    SmartDashboard.putNumber("Green", myGreen);
    SmartDashboard.putNumber("Blue", myBlue);

    SmartDashboard.putNumber("Proximity", proximity);
  //SmartDashboard.putNumber("IR", IR);

  if((myRed > 30)&& (myBlue < 0) && (myGreen < 0)){
    //Detected Red
    DetectedColor = "R";
  }
  if((myRed < 0)&& (myBlue > 10) && (myGreen > 20)){
    //Detected Green
    DetectedColor = "G";
  }
  if((myRed < 0)&& (myBlue > 40) && (myGreen < 0)){
    //Detected Blue
    DetectedColor = "B";
  }
  if((myRed > 3)&& (myBlue < 3) && (myGreen > 15)){
    //Detected Yellow
    DetectedColor = "Y";
  }
  if((myRed < 5)&& (myBlue < 20) && (myGreen < 5)){
    //Detected NA
    DetectedColor="U";
  }
  
  }
}
