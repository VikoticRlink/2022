// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter.loadBallSubcommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Shooter;


///////////////////////////////////////////////////////////////////////////////
/** A command that 'chambers' a ball by running the ball indexer until a ball
 *  reaches the limit sensor.
 * 
 *  NOTE: this command does nothing if no balls are detected in the shooter.
 */
public class ChamberBall extends CommandBase {
  Shooter m_shooterSubsystem;
  boolean m_isDone = false;

  ///////////////////////////////////////////////////////////////////////////////
  /** Creates an instance of the command
   * @param shooterSubsystem  Shooter subsystem used by the command
   */
  public ChamberBall(Shooter shooterSubsystem) {
    m_shooterSubsystem = shooterSubsystem;
    addRequirements(shooterSubsystem);
  }

  ///////////////////////////////////////////////////////////////////////////////
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  ///////////////////////////////////////////////////////////////////////////////
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_shooterSubsystem.numBallsDetected() > 0) {
      // Run the ball indexer to move ball(s) toward the top of the shooter
      m_shooterSubsystem.runBallIndexer(Shooter.BallIndexerMode.FeedBall);
    }
  }

  ///////////////////////////////////////////////////////////////////////////////
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // Stop the ball indexer
    m_shooterSubsystem.runBallIndexer(Shooter.BallIndexerMode.Stopped);
    m_isDone = true;
  }

  ///////////////////////////////////////////////////////////////////////////////
  /* Called after each time the scheduler runs the execute() method to determine
   * whether the command has finished.
   * 
   * Returns true when the ball has reached the limit sensor or if no balls
   * are detected anywhere in the shooter
   */
  @Override
  public boolean isFinished() {
    return (m_shooterSubsystem.numBallsDetected() < 1) ||
            m_shooterSubsystem.getBallLimitSensor() ||
            m_isDone;
  }
}