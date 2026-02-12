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
import frc.robot.commands.LaunchDrive;
import frc.robot.commands.IntakeDrive;
import frc.robot.commands.RobotCentricDrive;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LauncherSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
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

  private PIDController RotationPID = new PIDController(.006, 0, 0);
  

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
    RotationPID.enableContinuousInput(0, 360);
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

  m_driverController.a().onTrue(new ParallelRaceGroup(new RunCommand(() -> m_launcher.setSpeed(5000)),new CheckLaunchSpeed(m_launcher)).andThen(new RunCommand(() -> m_launcher.feed())));
  m_driverController.b().onTrue(new ParallelRaceGroup(new RunCommand(() -> m_launcher.setSpeed(2500)),new CheckLaunchSpeed(m_launcher)).andThen(new RunCommand(() -> m_launcher.feed())));
  m_driverController.pov(0).onTrue(new ParallelRaceGroup(new RunCommand(() -> m_launcher.setSpeed(3000)),new CheckLaunchSpeed(m_launcher)).andThen(new RunCommand(() -> m_launcher.feed())));
  m_driverController.pov(90).onTrue(new ParallelRaceGroup(new RunCommand(() -> m_launcher.setSpeed(3500)),new CheckLaunchSpeed(m_launcher)).andThen(new RunCommand(() -> m_launcher.feed())));
  m_driverController.pov(180).onTrue(new ParallelRaceGroup(new RunCommand(() -> m_launcher.setSpeed(4000)),new CheckLaunchSpeed(m_launcher)).andThen(new RunCommand(() -> m_launcher.feed())));
  m_driverController.pov(270).onTrue(new ParallelRaceGroup(new RunCommand(() -> m_launcher.setSpeed(4500)),new CheckLaunchSpeed(m_launcher)).andThen(new RunCommand(() -> m_launcher.feed())));
  m_driverController.x().onTrue(new ParallelRaceGroup(new RunCommand(() -> m_launcher.setSpeed(0)),new CheckLaunchSpeed(m_launcher)).andThen(new RunCommand(() -> m_launcher.feedOff())));
  //m_driverController.start().onTrue(new ParallelCommandGroup(new RunCommand(() -> m_launcher.feedOff()), new RunCommand(() -> m_launcher.setSpeed(0), m_launcher)));
  m_driverController.leftBumper().onTrue(new FieldCentricDrive(m_robotDrive, m_driverController));
  m_driverController.rightBumper().onTrue(new RobotCentricDrive(m_robotDrive, m_driverController));
  m_driverController.rightTrigger().onTrue(new LaunchDrive(m_robotDrive, m_driverController, m_launcher, RotationPID));
  m_driverController.rightTrigger().onFalse(new ParallelCommandGroup(new FieldCentricDrive(m_robotDrive, m_driverController), new RunCommand(() -> m_launcher.feedOff()), new RunCommand(() -> m_launcher.setSpeed(0.0))));
  //m_driverController.x().whileTrue(new RunCommand(() -> m_robotDrive.setX(), m_robotDrive));
  //m_driverController.back().whileTrue(new RunCommand(() -> m_robotDrive.zeroHeading(), m_robotDrive));
  //m_driverController.a().onTrue(new InstantCommand(() -> m_intake.changeSetpoint(.3), m_intake));
  //m_driverController.b().onTrue(new InstantCommand(() -> m_intake.changeSetpoint(0), m_intake));
  m_driverController.leftTrigger().onTrue(new ParallelCommandGroup(new RunCommand(() -> m_intake.setPercent(1), m_intake), new IntakeDrive(m_robotDrive, m_driverController, RotationPID)));
  m_driverController.leftTrigger().onFalse(new ParallelCommandGroup(new RunCommand(() -> m_intake.setPercent(0), m_intake), new FieldCentricDrive(m_robotDrive, m_driverController)));
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
