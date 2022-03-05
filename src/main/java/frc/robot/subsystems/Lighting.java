// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;

import java.awt.Color;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.util.Color8Bit;

public class Lighting extends SubsystemBase {
  /** Creates a new Lighting. */
  private static AddressableLED m_led;
  private static AddressableLEDBuffer m_ledBuffer;
  private static int m_rainbowFirstPixelHue;
  private static int l_shootColor;
  private static int currentColor;
  static int iPos=0;
  private static Color8Bit AllianceColor;

  public Lighting() {  
    m_led = new AddressableLED(0);
    m_ledBuffer = new AddressableLEDBuffer(120);
    m_led.setLength(m_ledBuffer.getLength());
    m_led.setData(m_ledBuffer);
    m_led.start();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (RobotContainer.isRedAlliance){
      AllianceColor = new Color8Bit(255,0,0);
    }else{
      AllianceColor = new Color8Bit(0,0,255);
    }
    if(RobotContainer.RobotShooting){
      ShootBall();
    }
    
    if(RobotState.isAutonomous()){
      LEDRY();
    }
    if(RobotState.isDisabled()){
      //All_LEDRainbow();
      
      switch(RobotContainer.m_chooser.getSelected().getName()){
        case "AutoRedOne":
          AutoColor(true, 1);
          break;
        case "AutoRedTwo":
          AutoColor(true, 2);
          break;
        case "AutoRedThree":
          AutoColor(true, 3);
          break;
        case "AutoBlueOne":
          AutoColor(false, 1);
          break;
        case "AutoBlueTwo":
          AutoColor(false, 2);
          break;
        case "AutoBlueThree":
          AutoColor(false, 3);
          break;
        case "Auto":
          AllGreen();
          break;
      }
    }
    
    if(RobotState.isEnabled() && RobotState.isTeleop()){
      All_LEDRainbow();
      if(RobotContainer.ManualControl){
        ShootBall();
      }
    }
  }
  private void AllGreen(){
    for (var i =0; i< m_ledBuffer.getLength(); i++){
      m_ledBuffer.setRGB(i, 0, 255, 0);
    }
    m_led.setData(m_ledBuffer);
  }
  private void LEDRY(){
      
    for (var i = 0; i < m_ledBuffer.getLength(); i++) {
      if (i % 2 == 0)
        m_ledBuffer.setRGB(i, 255, 150, 0);
      else
        m_ledBuffer.setRGB(i, 255, 0, 0);
      }     
    m_led.setData(m_ledBuffer);
  }
  private void All_LEDRainbow(){
    //--- make a rainbow pattern on LEDs ---//
    int ShowLEDs = m_ledBuffer.getLength();
    for (var i = 0; i < ShowLEDs; i++) {
        final var hue = (m_rainbowFirstPixelHue + (i * 180 / m_ledBuffer.getLength())) % 180;
        m_ledBuffer.setHSV(i, hue, 255, 128);
      }
      m_rainbowFirstPixelHue += 3;
      m_rainbowFirstPixelHue %= 180;
    m_led.setData(m_ledBuffer);
  }
  private void AutoColor(boolean RedSide, int whichAuto){
    Color8Bit MyLight = new Color8Bit(0, 0, 255);
    final Color8Bit BlackLight = new Color8Bit(0, 0, 0);
    boolean[][] colorpatern = {{true, false, false, false, false},{true, true, false, false, false},{true, true, true, false, false}};
    int mycounter=0;
    if(RedSide){
      MyLight = new Color8Bit(255, 0, 0);
    }
  
    for (var i = 0; i < m_ledBuffer.getLength(); i++) {
      if(colorpatern[whichAuto-1][mycounter]){
          m_ledBuffer.setLED(i, MyLight);
        }else{
          m_ledBuffer.setLED(i, BlackLight);
        }
        mycounter++;
        if(mycounter>=5){mycounter=0;}
      }
      
    m_led.setData(m_ledBuffer);
  }
  private void ShootBall(){
    int i_ledLength = 20;
    int i_ledStartA = 30;
    int i_ledStartB = 50;
    int ballsize = 4;
    //int deltaHue = 4;
    ClearBuffer(i_ledStartA,i_ledLength);
    ClearBuffer(i_ledStartB,i_ledLength);
    if (iPos<i_ledLength-ballsize){
        for (var i=0;i<ballsize;i++)
        {
          m_ledBuffer.setLED(iPos+i+i_ledStartA, AllianceColor);
          m_ledBuffer.setLED(iPos+i+i_ledStartB, AllianceColor);
         
        }
        m_led.setData(m_ledBuffer); 
        iPos++;
      }else{iPos=0;}
  }
  private void ClearBuffer(int startpoint, int stringlength){
    
    for (int i=0;i<stringlength;i++){
      m_ledBuffer.setRGB(startpoint+i, 0, 0, 0);
    }
  }
  
}
/*
Lighting mode ideas:
Auto settings - Chasing, matches Aliance color and auto number
Auto running - Chasing red/yellow?
Teleop - Rainbow
Manual controls - all white
Endgame mode - Alternating red/yellow? Red/Yellow up and down meter?

Postions:
Shooter, lights up in shooting motion when run
Along wheels, follows movement direction?
*/