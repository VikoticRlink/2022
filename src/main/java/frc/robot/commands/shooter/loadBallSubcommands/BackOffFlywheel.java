///////////////////////////////////////////////////////////////////////////////////////////////////
// Copyright (c) 2022 FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
///////////////////////////////////////////////////////////////////////////////////////////////////

/*-----------------------------------------------------------------------------\
|                                                                              |
|                       ================================                       |
|                       **    TEAM 5290 - Vikotics    **                       |
|                       ================================                       |
|                                                                              |
|                            °        #°                                       |
|                            *O       °@o                                      |
|                            O@ °o@@#° o@@                                     |
|                           #@@@@@@@@@@@@@@                                    |
|                           @@@@@@@@@@@@@@@                                    |
|                           @@@@@@@@@@@@@@°                                    |
|                             #@@@@@@@@@@@@@O....   .                          |
|                             o@@@@@@@@@@@@@@@@@@@@@o                          |
|                             O@@@@@@@@@@@@@@@@@@@#°                    *      |
|                             O@@@@@@@@@@@@@@@@@@@@@#O                O@@    O |
|                            .@@@@@@@@°@@@@@@@@@@@@@@@@#            °@@@    °@@|
|                            #@@O°°°°  @@@@@@@@@@@@@@@@@@°          @@@#*   @@@|
|                         .#@@@@@  o#oo@@@@@@@@@@@@@@@@@@@@@.       O@@@@@@@@@@|
|                        o@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@°     @@@@@@@@@°|
|                        @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@   .@@@@@o°   |
|          °***          @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@  @@@@@o     |
|     o#@@@@@@@@@@@@.   *@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@o@@@@@@      |
|OOo°@@@@@@@@@@@@O°#@#   @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@       |
|@@@@@@@@@@@@@@@@    o°  .@@@@@@@@@@@@@@@@@@@@@@@@#*@@@@@@@@@@@@@@@@@@@@       |
|@@@@@@@@@@@@@@@*         O@@@@@@@@@@@@@@@@@@@@@@@   °@@@@@@@@@@@@@@@@@@o      |
|@@@@#@@@@@@@@@            @@@@@@@@@@@@@@@@@@@@@@       .*@@@@@@@@@@@@@@.      |
|@@@°      @@@@O           @@@@@@@@@@@@@@@@@@@@o           °@@@@@@@@@@@o       |
|          @@@@@          .@@@@@@@@@@@@@@@@@@@*               O@@@@@@@*        |
|           @@@@@        o@@@@@@@@@@@@@@@@@@@@.               #@@@@@O          |
|           *@@@@@@@*  o@@@@@@@@@@@@@@@@@@@@@@°              o@@@@@            |
|           @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@.              @@@@@#            |
|          @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@O             #@@@@@             |
|          .@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@#           .@@@@@°             |
|           @@@@@@@@@@O*    @@@@@@@@@@@@@@@@@@@@@°         °O@@@°              |
|            °O@@@@@@       @@@@@@@@@@@@@@@@@@@@@@@                            |
|              o@@@@@°      @@@@@@@@@@@@@@@@@@@@@@@@                           |
|               @@@@@@.     @@@@@@@@@@@@@@@@@@@@@@@@@o                         |
|                @@@@@@*    @@@@@@@@@@@@@@@@@@@@@@@@@@                         |
|                o@@@@@@.  o@@@@@@@@@@@@@@@@@@@@@@@@@@@                        |
|                 #@@@@@@  *@@@@@@@@@@@@@@@@@@@@@@@@@@@@                       |
|                  °***    @@@@@@@@@@@@@@@@@@@@@@@@@@@@@O                      |
|                         .OOOOOOOOOOOOOOOOOOOOOOOOOOOOOO                      |
\-----------------------------------------------------------------------------*/

package frc.robot.commands.shooter.loadBallSubcommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.runtimeState.BotStateSubsystem;

///////////////////////////////////////////////////////////////////////////////
/**
 * A command used to back off a ball that has been moved up to the limit
 * switch by the ChamberBall comand, moving it backwards in the shooter, away
 * from the flywheels until it clears the limit sensor.
 * 
 * NOTE: this command does nothing if no balls are detected in the shooter.
 */
public class BackOffFlywheel extends CommandBase {
  BotStateSubsystem m_botState;
  ShooterSubsystem m_shooterSubsystem;
  // int SpinNumber = 0;

  /////////////////////////////////////////////////////////////////////////////
  /**
   * Creates an instance of the command
   * 
   * @param shooterSubsystem Shooter subsystem used by the command
   */
  public BackOffFlywheel(BotStateSubsystem botState, ShooterSubsystem shooterSubsystem) {
    m_botState = botState;
    m_shooterSubsystem = shooterSubsystem;
    addRequirements(shooterSubsystem);
  }

  /////////////////////////////////////////////////////////////////////////////
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // SpinNumber = 0;
    m_shooterSubsystem.resetIndexEncoder();
  }

  /////////////////////////////////////////////////////////////////////////////
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    /*
     * if (m_shooterSubsystem.numBallsDetected() > 0) {
     * m_shooterSubsystem.runBallIndexer(Shooter.BallIndexerMode.Reverse);
     * SpinNumber += 1;
     * }
     */
    if (m_botState.StealthMode == false) {
      m_botState.RobotShooting = true; // Set flag indicating we are shooting
    }
    m_shooterSubsystem.runBallIndexer(ShooterSubsystem.BallIndexerMode.Backoff);
  }

  /////////////////////////////////////////////////////////////////////////////
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_shooterSubsystem.runBallIndexer(ShooterSubsystem.BallIndexerMode.Stopped);
  }

  ///////////////////////////////////////////////////////////////////////////////
  /*
   * Called after each time the scheduler runs the execute() method to determine
   * whether the command has finished.
   * 
   * Returns true when the ball has backed off sufficiently from the limit
   * sensor or if no balls are detected anywhere in the shooter
   */
  @Override
  public boolean isFinished() {
    return m_shooterSubsystem.IndexAtLocation();
    // return (m_shooterSubsystem.getBallLimitSensor() == false);
    // return (SpinNumber == 8);
    // return (m_shooterSubsystem.numBallsDetected() < 1) ||
    // (m_shooterSubsystem.getBallLimitSensor() == false);
  }
}