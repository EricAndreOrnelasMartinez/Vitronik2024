// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.AMP;
import frc.robot.commands.Autos;
import frc.robot.commands.Down;
import frc.robot.commands.Drive;
import frc.robot.commands.DriveInvert;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.Humman;
import frc.robot.commands.OutTake;
import frc.robot.commands.Speaker;
import frc.robot.commands.TakeIntake;
import frc.robot.commands.TurnLeft;
import frc.robot.commands.TurnRight;
import frc.robot.commands.Up;
import frc.robot.subsystems.Chassis;
import frc.robot.subsystems.Elevador;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  private Chassis m_Chassis;
  private Shooter m_Shooter;
  private Intake m_Intake;
  private Elevador m_Elevador;

  // Replace with CommandPS4Controller or CommandJoystick if needed
  public static CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);

  public static CommandXboxController m_mechanismsController =
      new CommandXboxController(OperatorConstants.kMechanismsControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    m_Chassis = new Chassis();
    m_Shooter = new Shooter();
    m_Intake = new Intake();
    m_Elevador = new Elevador();
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    new Trigger(m_exampleSubsystem::exampleCondition)
        .onTrue(new ExampleCommand(m_exampleSubsystem));

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.

    //Drive
    m_driverController.rightBumper().whileTrue(new Drive(m_Chassis));
    m_driverController.leftBumper().whileTrue(new DriveInvert(m_Chassis));
    m_driverController.x().whileTrue(new TurnLeft(m_Chassis));
    m_driverController.b().whileTrue(new TurnRight(m_Chassis));

    //Mecanismos
    m_mechanismsController.b().whileTrue(new Speaker(m_Shooter));
    m_mechanismsController.a().whileTrue(new AMP(m_Shooter));
    m_mechanismsController.y().whileTrue(new Humman(m_Shooter));
    m_mechanismsController.x().whileTrue(new TakeIntake(m_Intake));
    m_mechanismsController.rightStick().whileTrue(new OutTake(m_Intake));
    m_mechanismsController.rightBumper().whileTrue(new Up(m_Elevador));
    m_mechanismsController.leftBumper().whileTrue(new Down(m_Elevador));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return Autos.exampleAuto(m_exampleSubsystem);
  }
}
