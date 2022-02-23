// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Shooter;



public class FeedFlywheel extends CommandBase {
  /** Creates a new FeedFlywheel. */
  
  private final Shooter m_shooter;
  private int m_countdown;
  private boolean m_lastLimitSwitch;
  private boolean m_thisLimitSwitch;

  public FeedFlywheel(Shooter shooter_subsystem) {
    m_shooter = shooter_subsystem;
    m_countdown = 2;
    m_lastLimitSwitch = false;

    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
   
    m_shooter.Run_Index_Motor(1);
    m_thisLimitSwitch = m_shooter.Limitswitch_Is_Closed();
    if (m_lastLimitSwitch != m_thisLimitSwitch)
    {
      m_countdown = m_countdown - 1;
    }
    m_lastLimitSwitch = m_thisLimitSwitch;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    m_shooter.Run_Index_Motor(0);
    m_shooter.Run_Flywheel_Motor(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_countdown == 0;
  }
}
