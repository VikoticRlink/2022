// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Shooter;
//import frc.robot.Constants;
import frc.robot.commands.shooter.loadBallSubcommands.*;

/////////////////////////////////////////////////////////////////////////////////
/** A command that chambers a ball and gets it ready to fire 
 *
 *  This command sequentially executes the ChamberBall and BackOffFlywheel commands 
 */
public class LoadBall extends SequentialCommandGroup {
  Shooter m_shooterSubsystem;

  ///////////////////////////////////////////////////////////////////////////////
  /** Creates an instance of the command
   * @param shooterSubsystem  Shooter subsystem the command will use
   */
  public LoadBall(Shooter shooterSubsystem) {
    m_shooterSubsystem = shooterSubsystem;
    addCommands(
        //new ChamberBall(shooterSubsystem)
        //  .withTimeout(Constants.ShooterConstants.kChamberBallTimeoutSeconds),
        new BackOffFlywheel(shooterSubsystem)
        );
  }
}