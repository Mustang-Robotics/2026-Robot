// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.CheckLaunchSpeed;
import frc.robot.commands.FieldCentricDrive;
import frc.robot.commands.HubDrive;
import frc.robot.commands.IntakeDrive;
import frc.robot.commands.RobotCentricDrive;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LauncherSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;


import com.pathplanner.lib.auto.AutoBuilder;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();
  private final LauncherSubsystem m_launcher = new LauncherSubsystem();
  private final IntakeSubsystem m_intake = new IntakeSubsystem();

  // The driver's controller
  private final CommandXboxController m_driverController = new CommandXboxController(OIConstants.kDriverControllerPort);

  private final SendableChooser<Command> m_chooser;

  private PIDController IntakeDrivePID = new PIDController(.006, 0, 0);
  

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
    IntakeDrivePID.enableContinuousInput(0, 360);
    m_chooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", m_chooser);
    // Configure default commands
    m_robotDrive.setDefaultCommand(new FieldCentricDrive(m_robotDrive, m_driverController));
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
   * subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling
   * passing it to a
   * {@link JoystickButton}.
   */
  private void configureButtonBindings() {

  m_driverController.a().onTrue(new ParallelCommandGroup(new RunCommand(() -> m_launcher.setSpeed(2500), m_launcher),new RunCommand(() -> m_launcher.feed())));
  m_driverController.b().onTrue(new ParallelCommandGroup(new RunCommand(() -> m_launcher.feedOff()), new RunCommand(() -> m_launcher.setSpeed(0), m_launcher)));
  m_driverController.x().onTrue(new FieldCentricDrive(m_robotDrive, m_driverController));
  m_driverController.rightBumper().onTrue(new RobotCentricDrive(m_robotDrive, m_driverController));
  m_driverController.leftBumper().onTrue(new HubDrive(m_robotDrive, m_driverController, IntakeDrivePID));
  m_driverController.start().whileTrue(new RunCommand(() -> m_robotDrive.setX(), m_robotDrive));
  m_driverController.back().whileTrue(new RunCommand(() -> m_robotDrive.zeroHeading(), m_robotDrive));
  m_driverController.rightTrigger().onTrue(new ParallelCommandGroup(new RunCommand(() -> m_intake.setPercent(.5), m_intake), new IntakeDrive(m_robotDrive, m_driverController, IntakeDrivePID)));
  m_driverController.rightTrigger().onFalse(new ParallelCommandGroup(new RunCommand(() -> m_intake.setPercent(0), m_intake), new FieldCentricDrive(m_robotDrive, m_driverController)));
   }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return m_chooser.getSelected();
  }
}
