// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class Climber extends SubsystemBase {
  /** Creates a new Climber. */
  WPI_TalonFX ClimbMaster = new WPI_TalonFX(Constants.MotorID.climberMaster);
  WPI_TalonFX ClimbSlave = new WPI_TalonFX(Constants.MotorID.climberSlave);


  public Climber() {
    ClimbSlave.setInverted(true);
    ClimbSlave.follow(ClimbMaster);
    ClimbSlave.setNeutralMode(NeutralMode.Brake);
    ClimbMaster.setNeutralMode(NeutralMode.Brake);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (RobotContainer.robotIsInManualTeleOpMode()) {
        double climbAmount = RobotContainer.operatorController.leftStickY();
        climbAmount *= Constants.MotorScaler.kClimberSpeed;
        ClimbMaster.set(TalonFXControlMode.PercentOutput, -1 * climbAmount);
  }
}
}
