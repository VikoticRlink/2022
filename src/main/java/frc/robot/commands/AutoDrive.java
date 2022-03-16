// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.autonomous.driveStages.DriveStage0;

import frc.robot.subsystems.DriveBaseNew;
import frc.robot.subsystems.Intake;
import frc.robot.commands.autonomous.driveStages.*;

public class AutoDrive extends SequentialCommandGroup {
  
  public AutoDrive(DriveBaseNew driveBaseSubsystem, Intake intakeSubsystem) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new DriveStage0(driveBaseSubsystem, intakeSubsystem),
      new DriveStage1(driveBaseSubsystem, intakeSubsystem),
      new DriveStage2(driveBaseSubsystem)
    );
  }
}
