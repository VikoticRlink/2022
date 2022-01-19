// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveBase extends SubsystemBase {
  /** Creates a new DriveBase. */
  public Integer leftDriveEncoder;
  public Integer rightDriveEncoder;
  private Integer scalingfactor=2;

  public DriveBase() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run I did some more changes.
  }
  public void domystuff(){}
  public Boolean dobetterstuff(Integer left, Integer Right){
    return false;
  }
}
