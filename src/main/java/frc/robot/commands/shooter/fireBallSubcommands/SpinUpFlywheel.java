// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter.fireBallSubcommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Shooter;


///////////////////////////////////////////////////////////////////////////////
/** A command used to run the flywheel until it has attained the desired speed
 * 
 *  NOTE: If one or more balls are present in the Shooter, this command will
 *  execute forever.  It must be scheduled using a timeout decorator.
 */
public class SpinUpFlywheel extends CommandBase {
  Shooter m_shooterSubsystem;

  /////////////////////////////////////////////////////////////////////////////
  /** Creates an instance of the command
   * @param shooterSubsystem  Shooter subsystem used by the command
   */
  public SpinUpFlywheel(Shooter shooterSubsystem) {
    m_shooterSubsystem = shooterSubsystem;
    addRequirements(shooterSubsystem);
  }

  /////////////////////////////////////////////////////////////////////////////
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  /////////////////////////////////////////////////////////////////////////////
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_shooterSubsystem.runFlywheel(true);
  }

  /////////////////////////////////////////////////////////////////////////////
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  /////////////////////////////////////////////////////////////////////////////
  // Only return true if no balls are detected in the Shooter
  @Override
  public boolean isFinished() 
  { 
    return (m_shooterSubsystem.numBallsDetected() < 1);
  }
}