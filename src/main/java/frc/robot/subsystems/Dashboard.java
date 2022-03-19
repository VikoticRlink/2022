// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.  subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;

import java.util.Map;

import javax.print.FlavorException;

import frc.robot.RobotContainer;
import frc.robot.subsystems.Shooter.FlywheelSpeed;
public class Dashboard extends SubsystemBase {
  private Shooter m_shooterSubsystem;
  private ShuffleboardTab m_shooterTab;
   private NetworkTableEntry[] m_shooterSpeedNTE;

  /** Creates a new Dashboard. */
  public Dashboard(Shooter shooterSubsystem) {    
    //SmartDashboard.putData("Fire Ball", new LoadAndFire(Shooter.FlywheelSpeed.Autonomous, RobotContainer.m_Shooter, null));
    //SmartDashboard.putData("DriveStage 0", new DriveStage0(RobotContainer.m_DriveBase, RobotContainer.m_Intake));
   
    //SmartDashboard.putData("Fire Ball", new FireBall(RobotContainer.m_Shooter));
    //SmartDashboard.putData("Intake Ball", new IntakeBall(RobotContainer.m_Intake));
    //SmartDashboard.putData("BackOffFlywheel", new BackOffFlywheel(RobotContainer.m_Shooter));
    // SEE: https://docs.wpilib.org/en/stable/docs/software/dashboards/smartdashboard/choosing-an-autonomous-program-from-smartdashboard.html
    m_shooterSubsystem = shooterSubsystem;
    m_shooterTab = Shuffleboard.getTab("Shooter");
    m_shooterSpeedNTE = new NetworkTableEntry[FlywheelSpeed.kNumSpeeds.value()];
    m_shooterSpeedNTE[FlywheelSpeed.Low.value()] =
      m_shooterTab.add("Speeds.Low", m_shooterSubsystem.getFlywheelSpeedValue(FlywheelSpeed.Low))
        .withWidget(BuiltInWidgets.kNumberSlider)
        .withProperties(Map.of("min", 0.0, "max", 1.0)) // specify widget properties here
        .getEntry();
    m_shooterSpeedNTE[FlywheelSpeed.Medium.value()] =
      m_shooterTab.add("Speeds.Medium", m_shooterSubsystem.getFlywheelSpeedValue(FlywheelSpeed.Medium))
        .withWidget(BuiltInWidgets.kNumberSlider)
        .withProperties(Map.of("min", 0.0, "max", 1.0)) // specify widget properties here
        .getEntry();
    m_shooterSpeedNTE[FlywheelSpeed.GreasedLightning.value()] =
      m_shooterTab.add("Speeds.GreasedLightning", m_shooterSubsystem.getFlywheelSpeedValue(FlywheelSpeed.GreasedLightning))
        .withWidget(BuiltInWidgets.kNumberSlider)
        .withProperties(Map.of("min", 0.0, "max", 1.0)) // specify widget properties here
        .getEntry();
    m_shooterSpeedNTE[FlywheelSpeed.Autonomous.value()] =
      m_shooterTab.add("Speeds.Autonomous", m_shooterSubsystem.getFlywheelSpeedValue(FlywheelSpeed.Autonomous))
      .withWidget(BuiltInWidgets.kNumberSlider)
      .withProperties(Map.of("min", 0.0, "max", 1.0)) // specify widget properties here
      .getEntry();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // We should look at Shuffleboard as our smartdashboard.
    // https://docs.wpilib.org/en/stable/docs/software/dashboards/shuffleboard/getting-started/shuffleboard-tour.html

    //SmartDashboard.putString("Auto chooser", RobotContainer.m_chooser.getSelected().getName());
    SmartDashboard.putBoolean("Drive Direction", RobotContainer.DriveDirection == RobotContainer.RobotDirection.Forward);
    SmartDashboard.putBoolean("Manual Mode", RobotContainer.ManualControl);
    SmartDashboard.putBoolean("Alliance", RobotContainer.isRedAlliance);
    
    SmartDashboard.putBoolean("First Ball", RobotContainer.m_Shooter.upperBallIsPresent());
    SmartDashboard.putBoolean("Second Ball", RobotContainer.m_Shooter.lowerBallIsPresent());
    SmartDashboard.putBoolean("Ball Against Flywheel", RobotContainer.m_Shooter.getBallLimitSensor());
    //SmartDashboard.putNumber("Bot Heading", RobotContainer.m_DriveBase.getHeading());
    //SmartDashboard.putNumber("Intake Arm Position", RobotContainer.m_Intake.getIntakeArmPositionRaw());
    //SmartDashboard.putNumber("T-rex Arm position", RobotContainer.m_tRex.getArmPositionRaw());

    //SmartDashboard.putNumber("leftDrive", RobotContainer.m_DriveBase.leftMaster.getSelectedSensorPosition());
    //SmartDashboard.putNumber("rightDrive", RobotContainer.m_DriveBase.rightMaster.getSelectedSensorPosition());

    // Get shooter speeds from the dashboard
    double low = m_shooterSpeedNTE[FlywheelSpeed.Low.value()].getDouble(m_shooterSubsystem.getFlywheelSpeedValue(FlywheelSpeed.Low));
    double med = m_shooterSpeedNTE[FlywheelSpeed.Medium.value()].getDouble(m_shooterSubsystem.getFlywheelSpeedValue(FlywheelSpeed.Medium));
    double gl = m_shooterSpeedNTE[FlywheelSpeed.GreasedLightning.value()].getDouble(m_shooterSubsystem.getFlywheelSpeedValue(FlywheelSpeed.GreasedLightning));
    double aut = m_shooterSpeedNTE[FlywheelSpeed.Autonomous.value()].getDouble(m_shooterSubsystem.getFlywheelSpeedValue(FlywheelSpeed.Autonomous));
    m_shooterSubsystem.setFlywheelSpeedValue(FlywheelSpeed.Low, low);
    m_shooterSubsystem.setFlywheelSpeedValue(FlywheelSpeed.Medium, med);
    m_shooterSubsystem.setFlywheelSpeedValue(FlywheelSpeed.GreasedLightning, gl);
    m_shooterSubsystem.setFlywheelSpeedValue(FlywheelSpeed.Autonomous, aut);
    
    //PushMotorTemps();
  }

  //Motor Temp records
  //private void PushMotorTemps(){
    //SmartDashboard.putNumber("Left Drive Temp", RobotContainer.m_DriveBase.leftMaster.getTemperature());
    //SmartDashboard.putNumber("Left Drive Slave Temp", RobotContainer.m_DriveBase.leftSlave.getTemperature());
    //SmartDashboard.putNumber("Right Drive Temp", RobotContainer.m_DriveBase.rightMaster.getTemperature());
    //SmartDashboard.putNumber("Right Drive Slave Temp", RobotContainer.m_DriveBase.rightSlave.getTemperature());
  //  SmartDashboard.putNumber("Climb Master Temp", RobotContainer.m_Climber.ClimbMaster.getTemperature());
  //  SmartDashboard.putNumber("Climb Slave Temp", RobotContainer.m_Climber.ClimbSlave.getTemperature());
  //  SmartDashboard.putNumber("Intake Actuator Temp", RobotContainer.m_Intake.m_armMotor.getTemperature());
  //  SmartDashboard.putNumber("tRex Master Temp", RobotContainer.m_tRex.m_tRexMaster.getTemperature());
  //  SmartDashboard.putNumber("tRex Slave Temp", RobotContainer.m_tRex.m_tRexSlave.getTemperature());
   
  //}
}

