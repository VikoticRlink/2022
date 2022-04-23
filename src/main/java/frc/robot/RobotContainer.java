// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.driveBase.DriveBaseNew;
import frc.robot.subsystems.lighting.Lighting;
import frc.robot.subsystems.Shooter.FlywheelSpeed;
import frc.robot.subsystems.*;
import frc.robot.commands.autonomous.*;
import frc.robot.commands.*;
import frc.robot.utility.*;


/////////////////////////////////////////////////////////////////////////////
/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  /////////////////////////////////////////////////////////////////////////////
  /** Directions used when driving the robot */
  public enum RobotDirection {
    Forward,  /** Forward commands drive the robot forward normally */
    Reverse; /** Forward commands drive the robot in reverse */

    /** Get the human-readable name of the direction */
    @Override
    public String toString() { return this.name(); }
  };

  // Xbox controllers for driving the robot manually
  public static JoystickController driverController;   /** Xbox controller used by the robot driver */
  public static JoystickController operatorController; /** Xbox controller used by the robot operator */

  public static boolean ManualControl = false;
  public static RobotDirection DriveDirection = RobotDirection.Forward;
  public static boolean isRedAlliance = false;
  public static boolean RobotShooting = false;
  public static boolean StealthMode = false;

  // The robot's subsystems are defined here...
  //private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  public static Climber m_Climber = new Climber();
  public static DriveBaseNew m_DriveBase = new DriveBaseNew();
  public static Shooter m_Shooter = new Shooter();
  public static Intake m_Intake = new Intake(m_Shooter);
  public static Lighting m_Lighting = new Lighting();
  public static Dashboard m_Dashboard = new Dashboard();
  public static T_rex m_tRex = new T_rex();

  //The robot's commands are defined here...
  //private final ExampleCommand m_autoCommand = new ExampleCommand(m_exampleSubsystem);
  private final AutoDrive m_autonomous = new AutoDrive(m_DriveBase, m_Intake, m_Shooter);
  private final AutoRedOne Red1 = new AutoRedOne();
  private final AutoRedTwo Red2 = new AutoRedTwo();
  private final AutoRedThree Red3 = new AutoRedThree();
  private final AutoBlueOne Blue1 = new AutoBlueOne();
  private final AutoBlueTwo Blue2 = new AutoBlueTwo();
  private final AutoBlueThree Blue3 = new AutoBlueThree();
  private final AutoHighGoal m_AutoHighGoal = new AutoHighGoal(m_DriveBase, m_Intake, m_Shooter);
  public static SendableChooser<Command> m_chooser = new SendableChooser<>();
  public static SendableChooser<String> m_stealthMode = new SendableChooser<>();
  public static SendableChooser<String> m_NoLimits = new SendableChooser<>();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    
    configureButtonBindings();



    m_DriveBase.setDefaultCommand(new DriveByJoysticks());


    m_chooser.setDefaultOption("High Auto", m_AutoHighGoal); 
    m_chooser.addOption("Simple Auto", m_autonomous); 
    m_chooser.addOption("Red 1", Red1);
    m_chooser.addOption("Red 2", Red2);
    m_chooser.addOption("Red 3", Red3);
    m_chooser.addOption("Blue 1", Blue1);
    m_chooser.addOption("Blue 2", Blue2);
    m_chooser.addOption("Blue 3", Blue3);   
    SmartDashboard.putData(m_chooser);
m_NoLimits.setDefaultOption("Limits ON", "LimitsOn");
m_NoLimits.addOption("Limits OFF", "LimitsOff");
SmartDashboard.putData(m_NoLimits);
m_stealthMode.setDefaultOption("Stealth OFF", "Normal");
m_stealthMode.addOption("Stealth On", "Stealthy");
SmartDashboard.putData(m_stealthMode);
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    driverController = new JoystickController(0);
    driverController.leftStickYProc.sensitivity=0.3;
    driverController.rightStickYProc.sensitivity=0.3;

    operatorController = new JoystickController(1);
        // Map buttons on operator controller
        //operatorController.Start.whenPressed(new ManualModeToggle());
        operatorController.bumpLeft.whenHeld(new ManualMode());
        operatorController.bumpRight.whenHeld(new ManualMode());
        operatorController.A.whenPressed(new LoadAndFire(FlywheelSpeed.Low, m_Shooter, operatorController.A));
        operatorController.X.whenPressed(new LoadAndFire(FlywheelSpeed.Medium, m_Shooter, operatorController.X));
        operatorController.Y.whenPressed(new LoadAndFire(FlywheelSpeed.GreasedLightning, m_Shooter, operatorController.Y));
        operatorController.B.whenHeld(new IntakeBall(RobotContainer.m_Intake));
        
        driverController.Start.whenPressed(new ToggleFrontOfBot());
      }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return m_chooser.getSelected();
  }

  /** 
   * Returns true if the robot is being driven in Manual, tele-operated mode
   */
  public static boolean robotIsInManualTeleOpMode() {
    return (RobotState.isEnabled() && RobotState.isTeleop() && ManualControl);
  }
}
