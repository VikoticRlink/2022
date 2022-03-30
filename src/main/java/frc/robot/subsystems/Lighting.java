// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.utility.LEDPattern;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.util.Color8Bit;

public class Lighting extends SubsystemBase {

  //////////////////////////////////
  /// *** CONSTANTS ***
  //////////////////////////////////

  // TODO: The following LEDStrip objects need to be configured with
  //       the correct start index and count
  private static final LEDStrip kLowerLeftStrip = new LEDStrip(0, 33);
  private static final LEDStrip kUpperLeftStrip = new LEDStrip(33, 33);
  private static final LEDStrip kLowerRightStrip = new LEDStrip(66, 33);
  private static final LEDStrip kUpperRightStrip = new LEDStrip(99, 33);

  //private static final Color8Bit kOff = new Color8Bit(0, 0, 0);
  //private static final Color8Bit kGreen = new Color8Bit(0, 255, 0);
  private static final Color8Bit kRed = new Color8Bit(255, 0, 0);
  private static final Color8Bit kBlue = new Color8Bit(0, 0, 255);
  private static final Color8Bit kYellow = new Color8Bit(255, 150, 0);

  private static AddressableLED m_led;
  private static AddressableLEDBuffer m_ledBuffer;
  private static int m_rainbowFirstPixelHue;

  //private static int l_shootColor;
  //private static int currentColor;
  static int iPos=0;
  private static Color8Bit AllianceColor;
  private LEDPattern m_lowerLeftSweep;
  private LEDPattern m_upperLeftSweep;
  private LEDPattern m_lowerRightSweep;
  private LEDPattern m_upperRightSweep;


  /////////////////////////////////////////////////////////////////////////////
  /** Creates an instance of the subsystem
  */
  public Lighting() {
    m_led = new AddressableLED(1);
    m_ledBuffer = new AddressableLEDBuffer(132);
    m_lowerLeftSweep = new LEDPattern(kLowerLeftStrip.startIndex, kLowerLeftStrip.numLEDs, kYellow);
    m_upperLeftSweep = new LEDPattern(kUpperLeftStrip.startIndex, kUpperLeftStrip.numLEDs, kYellow);
    m_lowerRightSweep = new LEDPattern(kLowerRightStrip.startIndex, kLowerRightStrip.numLEDs, kYellow);
    m_upperRightSweep = new LEDPattern(kUpperRightStrip.startIndex, kUpperRightStrip.numLEDs, kYellow);
    m_led.setLength(m_ledBuffer.getLength());
    m_led.setData(m_ledBuffer);
    m_led.start();

    updateAllianceColor();
  }

  /////////////////////////////////////////////////////////////////////////////
  /** Updates the active alliance color */
  public void updateAllianceColor() {
    AllianceColor = (RobotContainer.isRedAlliance) ? kRed : kBlue;
    m_lowerLeftSweep.setColor(AllianceColor);
    m_upperLeftSweep.setColor(AllianceColor);
    m_lowerRightSweep.setColor(AllianceColor);
    m_upperRightSweep.setColor(AllianceColor);
  }

  /////////////////////////////////////////////////////////////////////////////
  /** Resets the LED sweeps */
  public void resetSweep() {
    m_lowerLeftSweep.reset();
    m_upperLeftSweep.reset();
    m_lowerRightSweep.reset();
    m_upperRightSweep.reset();
  }

  /////////////////////////////////////////////////////////////////////////////
  /** This method gets called once per scheduler run (about every 20ms) */ 
  @Override
  public void periodic() {
    updateAllianceColor();
    
    if(RobotState.isAutonomous()) {
      LEDRY();
    }

    if (RobotState.isDisabled()) {
      updateDisabled();
    }
    
    if(RobotState.isEnabled() && RobotState.isTeleop()){
      if (RobotContainer.RobotShooting) {
        ShootBall();
      }
      else {
        All_LEDRainbow();
      }
    }

    // Apply the LED buffer states to the LED strip
    m_led.setData(m_ledBuffer);
  }


  /////////////////////////////////////////////////////////////////////////////
  /** Update LED's when the robot is disabled */
  private void updateDisabled() {
    switch (RobotContainer.m_chooser.getSelected().getName()) {
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
        case "AutoDrive":
        AllYellow();
        break;      
        case "AutoHighGoal":
        All_LEDRainbow();
        break;
    }
  }

  /////////////////////////////////////////////////////////////////////////////
  /** Make all LED's yellow */
  private void AllYellow() {
    for (var i =0; i< m_ledBuffer.getLength(); i++){
      m_ledBuffer.setLED(i, kYellow);
    }
  }

  /////////////////////////////////////////////////////////////////////////////
  /** Make LED's an alternating sequence of red and yellow */
  private void LEDRY() {
    for (var i = 0; i < m_ledBuffer.getLength(); i++) {
      if (i % 2 == 0) {
        m_ledBuffer.setLED(i, kYellow);
      }
      else {
        m_ledBuffer.setLED(i, kRed);
      }
    }
  }

  /////////////////////////////////////////////////////////////////////////////
  /** Make LED's a rainbow pattern
   * @note Unicorns and sprinkles will be added in a future update...
   */
  private void All_LEDRainbow(){
    //--- make a rainbow pattern on LEDs ---//
    int ShowLEDs = m_ledBuffer.getLength();
    for (var i = 0; i < ShowLEDs; i++) {
      final var hue = (m_rainbowFirstPixelHue + (i * 180 / m_ledBuffer.getLength())) % 180;
      m_ledBuffer.setHSV(i, hue, 255, 128);
    }

    m_rainbowFirstPixelHue += 3;
    m_rainbowFirstPixelHue %= 180;
  }

  /////////////////////////////////////////////////////////////////////////////
  /** Update LED's when the robot is in Autonomous mode */
  private void AutoColor(boolean RedSide, int whichAuto){
    Color8Bit MyLight = new Color8Bit(0, 0, 255);
    final Color8Bit BlackLight = new Color8Bit(0, 0, 0);
    boolean[][] colorpatern = {{true, false, false, false, false},{true, true, false, false, false},{true, true, true, false, false}};
    int mycounter=0;
    if(RedSide){
      MyLight = new Color8Bit(255, 0, 0);
    }
  
    for (var i = 0; i < m_ledBuffer.getLength(); i++) {
      if (colorpatern[whichAuto-1][mycounter]) {
        m_ledBuffer.setLED(i, MyLight);
      }
      else {
        m_ledBuffer.setLED(i, BlackLight);
      }

      mycounter++;
      mycounter %= 5;
    }
      
  }

  /////////////////////////////////////////////////////////////////////////////
  private void ShootBall(){
    // Apply the current pattern
    m_lowerLeftSweep.process(m_ledBuffer);
    m_upperLeftSweep.process(m_ledBuffer);
    m_lowerRightSweep.process(m_ledBuffer);
    m_upperRightSweep.process(m_ledBuffer);
  }


  /////////////////////////////////////////////////////////////////////////////
  /** Helper class used to store a description of a strip of addressable
   *  LEDs
   */
  private static class LEDStrip {
    public final int startIndex, numLEDs;

    /** Creates an instance of the object
     * @param start   Start address (index) of the strip
     * @param count   Number of LED's in the strip
     */
    public LEDStrip(int start, int count) {
      startIndex = start;
      numLEDs = count;
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