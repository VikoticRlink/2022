// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Shooter;


// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class LoadBall extends SequentialCommandGroup {
  /** Creates a new LoadBall. */

public final Shooter m_shooter;



  public LoadBall(Shooter shooter_subsystem) {

    m_shooter = shooter_subsystem;

    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands( new ChamberBall(shooter_subsystem),
    new BackOffFlywheel(shooter_subsystem));
  }
}
