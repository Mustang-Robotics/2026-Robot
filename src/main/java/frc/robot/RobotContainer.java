// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.AutoLaunchDrive;
import frc.robot.commands.AutoLaunchDriveIntake;
import frc.robot.commands.CheckLaunchSpeed;
import frc.robot.commands.FieldCentricDrive;
import frc.robot.commands.IntakeLaunch;
import frc.robot.commands.LaunchDrive;
//import frc.robot.commands.OrientDrive;
//import frc.robot.commands.IntakeDrive;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.commands.RobotCentricDrive;
import frc.robot.commands.ZeroShooter;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LauncherSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import frc.robot.lib.BLine.*;
import edu.wpi.first.math.controller.PIDController;

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

  //private final SendableChooser<Command> m_chooser;
  private final SendableChooser<Command> m_chooser = new SendableChooser<>();

  private TrapezoidProfile.Constraints rotationSpeed = new TrapezoidProfile.Constraints(540, 720);
  private ProfiledPIDController RotationPID = new ProfiledPIDController(.03, 0, 0.001, rotationSpeed);
  private TrapezoidProfile.Constraints X_Speed = new TrapezoidProfile.Constraints(0.0, 0.0);
  private ProfiledPIDController X_PID = new ProfiledPIDController(0, 0, 0.0, X_Speed);
  private TrapezoidProfile.Constraints Y_Speed = new TrapezoidProfile.Constraints(0.0, 0.0);
  private ProfiledPIDController Y_PID = new ProfiledPIDController(0, 0, 0.0, Y_Speed);
  private TrapezoidProfile.Constraints Center_X_Speed = new TrapezoidProfile.Constraints(0, 0);
  private ProfiledPIDController Center_X_PID = new ProfiledPIDController(0, 0, 0, Center_X_Speed);
  private TrapezoidProfile.Constraints Center_Y_Speed = new TrapezoidProfile.Constraints(0, 0);
  private ProfiledPIDController Center_Y_PID = new ProfiledPIDController(0, 0, 0, Center_Y_Speed);

  
//BLine Path Follower  
FollowPath.Builder pathBuilder = new FollowPath.Builder(
    m_robotDrive,                      // The drive subsystem to require
    m_robotDrive::getPose,             // Supplier for current robot pose
    m_robotDrive::getRobotRelativeSpeeds,    // Supplier for current speeds
    m_robotDrive::driveRobotRelative,               // Consumer to drive the robot
    new PIDController(4.1, 0.0, 0.05),    // Translation PID
    new PIDController(7.0, 0.0, 0.0),    // Rotation PID
    new PIDController(2, 0.0, 0.0)     // Cross-track PID
).withDefaultShouldFlip()                // Auto-flip for red alliance
 .withPoseReset(m_robotDrive::resetOdometry);  // Reset odometry at path start

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    NamedCommands.registerCommand("IntakeOn", new InstantCommand(() -> m_intake.setPercent(1)));
    NamedCommands.registerCommand("IntakeOff", new InstantCommand(() -> m_intake.setPercent(0)));
    NamedCommands.registerCommand("LaunchOn", 
    new CheckLaunchSpeed(m_launcher)
    .andThen(
        new RunCommand(() -> {
            // 1. Get where we are now
            double current = m_intake.getSetpoint();
            
            // 2. Add a small step (e.g., 0.005) but stop at 0.25
            m_intake.changeSetpoint(Math.min(0.25, current + 0.0015));
            
            // 3. Keep the feeder running simultaneously
            m_launcher.feed();
        }, m_intake, m_launcher) // Claims both subsystems
    )
);
    NamedCommands.registerCommand("LaunchOff", new ZeroShooter(m_launcher, m_intake));
    NamedCommands.registerCommand("Intake Up", new InstantCommand(() -> m_intake.changeSetpoint(.34)));
    NamedCommands.registerCommand("SpinUp", new InstantCommand(() -> m_launcher.setSpeed(2700)));//3150)));
    FollowPath.registerEventTrigger("IntakeOn", IntakeOn);
    FollowPath.registerEventTrigger("IntakeOff", IntakeOff);
    FollowPath.registerEventTrigger("SpinUp", SpinUp);
    

    configureButtonBindings();
    RotationPID.enableContinuousInput(0, 360);
    //m_chooser = AutoBuilder.buildAutoChooser();
    m_chooser.setDefaultOption("none", new InstantCommand());
    m_chooser.addOption("Blue Right", BlueRightAuto());
    m_chooser.addOption("Blue Left", BlueLeftAuto());
    m_chooser.addOption("Red Right", RedRightAuto());
    m_chooser.addOption("Red Left", RedLeftAuto());
    m_chooser.addOption("Blue Center", BlueCenterAuto());
    m_chooser.addOption("Red Center", RedCenterAuto());
    m_chooser.addOption("Blue Pass Right", BluePassingAutoRight());
    m_chooser.addOption("Blue Pass Left", BluePassingAutoLeft());
    m_chooser.addOption("Red Pass Right", RedPassingAutoRight());
    m_chooser.addOption("Red Pass Left", RedPassingAutoLeft());
    m_chooser.addOption("Right Trench BLINE", RT());
    m_chooser.addOption("Left Trench BLINE", LT());
    m_chooser.addOption("Right Bump BLINE", RB());
    m_chooser.addOption("Left Bump BLINE", LB());
    m_chooser.addOption("Depot Bump BLINE", DB());
    m_chooser.addOption("Depot Trench BLINE", DT());
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
  //SmartDashboard.putNumber("Set RPM", 0.0);
  //m_driverController.y().onTrue(new ParallelRaceGroup(new RunCommand(() -> m_launcher.setSpeed(SmartDashboard.getNumber("Set RPM", 0.0))),new CheckLaunchSpeed(m_launcher)).andThen(new RunCommand(() -> m_launcher.feed())));
  //m_driverController.y().onFalse(new ParallelCommandGroup(new RunCommand(() -> m_launcher.feedOff()), new RunCommand(() -> m_launcher.setSpeed(0), m_launcher)));
  //m_driverController.b().onTrue(new ParallelRaceGroup(new RunCommand(() -> m_launcher.setSpeed(2500)),new CheckLaunchSpeed(m_launcher)).andThen(new RunCommand(() -> m_launcher.feed())));
  m_driverController.rightBumper().onTrue(new ParallelRaceGroup(new RunCommand(() -> m_launcher.setSpeed(3000)),new CheckLaunchSpeed(m_launcher)).andThen(new InstantCommand(() -> m_intake.changeSetpoint(.13))).andThen(new RunCommand(() -> m_launcher.feed())));
  //m_driverController.pov(0).onTrue(new OrientDrive(m_robotDrive, m_driverController, RotationPID, 0));
  //m_driverController.pov(0).onFalse(new FieldCentricDrive(m_robotDrive, m_driverController));
  //m_driverController.pov(180).onTrue(new OrientDrive(m_robotDrive, m_driverController, RotationPID, 180));
  //m_driverController.pov(180).onFalse(new FieldCentricDrive(m_robotDrive, m_driverController));
  //m_driverController.pov(270).onTrue(new ParallelRaceGroup(new RunCommand(() -> m_launcher.setSpeed(4500)),new CheckLaunchSpeed(m_launcher)).andThen(new RunCommand(() -> m_launcher.feed())));
  m_driverController.rightBumper().onFalse(new ZeroShooter(m_launcher, m_intake));
  //m_driverController.start().onTrue(new ParallelCommandGroup(new RunCommand(() -> m_launcher.feedOff()), new RunCommand(() -> m_launcher.setSpeed(0), m_launcher)));
  m_driverController.a().onTrue(new FieldCentricDrive(m_robotDrive, m_driverController));
  m_driverController.b().onTrue(new RobotCentricDrive(m_robotDrive, m_driverController));
  m_driverController.x().onTrue(new InstantCommand(() -> m_intake.changeSetpoint(.34)));
  m_driverController.rightTrigger().onTrue(new LaunchDrive(m_robotDrive, m_driverController, m_launcher, RotationPID, m_intake));
  m_driverController.rightTrigger().onFalse(new ParallelCommandGroup(new FieldCentricDrive(m_robotDrive, m_driverController), new ZeroShooter(m_launcher, m_intake)));
  //m_driverController.x().whileTrue(new RunCommand(() -> m_robotDrive.setX(), m_robotDrive));
  m_driverController.back().whileTrue(new RunCommand(() -> m_robotDrive.zeroHeading(), m_robotDrive));
  //m_driverController.a().onTrue(new InstantCommand(() -> m_intake.changeSetpoint(.3), m_intake));
  //m_driverController.b().onTrue(new InstantCommand(() -> m_intake.changeSetpoint(0), m_intake));
  m_driverController.leftTrigger().onTrue(new IntakeLaunch(m_robotDrive, m_driverController, m_launcher, RotationPID, m_intake));
  m_driverController.leftTrigger().onFalse(new InstantCommand(() -> m_intake.setPercent(0), m_intake).andThen(new ParallelCommandGroup(new FieldCentricDrive(m_robotDrive, m_driverController), new ZeroShooter(m_launcher, m_intake))));
  m_driverController.leftBumper().onTrue(new ParallelCommandGroup(new InstantCommand(() -> m_intake.changeSetpoint(0.005)), new RunCommand(() -> m_intake.setPercent(1), m_intake)));
  m_driverController.leftBumper().onFalse(new RunCommand(() -> m_intake.setPercent(0), m_intake));
  m_driverController.y().onTrue(new InstantCommand(() -> m_launcher.setSpeed(-1500)));
  m_driverController.y().onFalse(new ZeroShooter(m_launcher, m_intake));
  m_driverController.pov(0).onTrue(new InstantCommand(() -> m_launcher.feedReverse()));
  m_driverController.pov(0).onFalse(new InstantCommand(() -> m_launcher.feedOff()));
  m_driverController.pov(180).onTrue(new InstantCommand(() -> m_launcher.feed()));
  m_driverController.pov(180).onFalse(new InstantCommand(() -> m_launcher.feedOff()));
  m_driverController.pov(270).onTrue(Tune);

   }

  private Command BlueRightPath() {
    try{
        // Load the path you want to follow using its name in the GUI
        PathPlannerPath path = PathPlannerPath.fromPathFile("Right Side Loop 1");
        // Create a path following command using AutoBuilder. This will also trigger event markers.
        return AutoBuilder.followPath(path);
    } catch (Exception e) {
        DriverStation.reportError(e.getMessage(), e.getStackTrace());
        return Commands.none();
    }
  }

    private Command BlueRightPathTwo() {
    try{
        // Load the path you want to follow using its name in the GUI
        PathPlannerPath path = PathPlannerPath.fromPathFile("Right Side Loop 1");
        PathConstraints constraints = new PathConstraints(2, 4, Units.degreesToRadians(360), Units.degreesToRadians(480));
        // Create a path following command using AutoBuilder. This will also trigger event markers.
        return AutoBuilder.pathfindThenFollowPath(path, constraints);
    } catch (Exception e) {
        DriverStation.reportError(e.getMessage(), e.getStackTrace());
        return Commands.none();
    }
  }

  private Command BlueLeftPath() {
    try{
        // Load the path you want to follow using its name in the GUI
        PathPlannerPath path = PathPlannerPath.fromPathFile("Right Side Loop 1");
        // Create a path following command using AutoBuilder. This will also trigger event markers.
        return AutoBuilder.followPath(path.mirrorPath());
    } catch (Exception e) {
        DriverStation.reportError(e.getMessage(), e.getStackTrace());
        return Commands.none();
    }
  }

  private Command BlueLeftPathTwo() {
    try{
        // Load the path you want to follow using its name in the GUI
        PathPlannerPath path = PathPlannerPath.fromPathFile("Right Side Loop 1");
        PathConstraints constraints = new PathConstraints(2, 4, Units.degreesToRadians(360), Units.degreesToRadians(480));
        // Create a path following command using AutoBuilder. This will also trigger event markers.
        return AutoBuilder.pathfindThenFollowPath(path.mirrorPath(), constraints);
    } catch (Exception e) {
        DriverStation.reportError(e.getMessage(), e.getStackTrace());
        return Commands.none();
    }
  }

  private Command RedRightPath() {
    try{
        // Load the path you want to follow using its name in the GUI
        PathPlannerPath path = PathPlannerPath.fromPathFile("Right Side Loop 1");
        // Create a path following command using AutoBuilder. This will also trigger event markers.
        return AutoBuilder.followPath(path.flipPath());
    } catch (Exception e) {
        DriverStation.reportError(e.getMessage(), e.getStackTrace());
        return Commands.none();
    }
  }

  private Command RedRightPathTwo() {
    try{
        // Load the path you want to follow using its name in the GUI
        PathPlannerPath path = PathPlannerPath.fromPathFile("Right Side Loop 1");
        PathConstraints constraints = new PathConstraints(2, 4, Units.degreesToRadians(360), Units.degreesToRadians(480));
        // Create a path following command using AutoBuilder. This will also trigger event markers.
        return AutoBuilder.pathfindThenFollowPath(path.flipPath(), constraints);
    } catch (Exception e) {
        DriverStation.reportError(e.getMessage(), e.getStackTrace());
        return Commands.none();
    }
  }

  private Command RedLeftPath() {
    try{
        // Load the path you want to follow using its name in the GUI
        PathPlannerPath path = PathPlannerPath.fromPathFile("Right Side Loop 1");
        // Create a path following command using AutoBuilder. This will also trigger event markers.
        return AutoBuilder.followPath(path.flipPath().mirrorPath());
    } catch (Exception e) {
        DriverStation.reportError(e.getMessage(), e.getStackTrace());
        return Commands.none();
    }
  }

  private Command RedLeftPathTwo() {
    try{
        // Load the path you want to follow using its name in the GUI
        PathPlannerPath path = PathPlannerPath.fromPathFile("Right Side Loop 1");
        PathConstraints constraints = new PathConstraints(2, 4, Units.degreesToRadians(360), Units.degreesToRadians(480));
        // Create a path following command using AutoBuilder. This will also trigger event markers.
        return AutoBuilder.pathfindThenFollowPath(path.flipPath().mirrorPath(), constraints);
    } catch (Exception e) {
        DriverStation.reportError(e.getMessage(), e.getStackTrace());
        return Commands.none();
    }
  }

  private Command BlueCenterPath() {
    try{
        // Load the path you want to follow using its name in the GUI
        PathPlannerPath path = PathPlannerPath.fromPathFile("Center");
        // Create a path following command using AutoBuilder. This will also trigger event markers.
        return AutoBuilder.followPath(path);
    } catch (Exception e) {
        DriverStation.reportError(e.getMessage(), e.getStackTrace());
        return Commands.none();
    }
  }

  private Command RedCenterPath() {
    try{
        // Load the path you want to follow using its name in the GUI
        PathPlannerPath path = PathPlannerPath.fromPathFile("Center");
        // Create a path following command using AutoBuilder. This will also trigger event markers.
        return AutoBuilder.followPath(path.flipPath());
    } catch (Exception e) {
        DriverStation.reportError(e.getMessage(), e.getStackTrace());
        return Commands.none();
    }
  }

  private Command BluePassingPathRight() {
    try{
        // Load the path you want to follow using its name in the GUI
        PathPlannerPath path = PathPlannerPath.fromPathFile("Passing Path");
        // Create a path following command using AutoBuilder. This will also trigger event markers.
        return AutoBuilder.followPath(path);
    } catch (Exception e) {
        DriverStation.reportError(e.getMessage(), e.getStackTrace());
        return Commands.none();
    }
  }

  private Command BluePassingPathLeft() {
    try{
        // Load the path you want to follow using its name in the GUI
        PathPlannerPath path = PathPlannerPath.fromPathFile("Passing Path");
        // Create a path following command using AutoBuilder. This will also trigger event markers.
        return AutoBuilder.followPath(path.mirrorPath());
    } catch (Exception e) {
        DriverStation.reportError(e.getMessage(), e.getStackTrace());
        return Commands.none();
    }
  }

  private Command RedPassingPathRight() {
    try{
        // Load the path you want to follow using its name in the GUI
        PathPlannerPath path = PathPlannerPath.fromPathFile("Passing Path");
        // Create a path following command using AutoBuilder. This will also trigger event markers.
        return AutoBuilder.followPath(path.flipPath());
    } catch (Exception e) {
        DriverStation.reportError(e.getMessage(), e.getStackTrace());
        return Commands.none();
    }
  }

  private Command RedPassingPathLeft() {
    try{
        // Load the path you want to follow using its name in the GUI
        PathPlannerPath path = PathPlannerPath.fromPathFile("Passing Path");
        // Create a path following command using AutoBuilder. This will also trigger event markers.
        return AutoBuilder.followPath(path.flipPath().mirrorPath());
    } catch (Exception e) {
        DriverStation.reportError(e.getMessage(), e.getStackTrace());
        return Commands.none();
    }
  }

  private Command BlueRightAuto() {
    return new SequentialCommandGroup(
      new InstantCommand(() -> m_robotDrive.resetOdometry(AutoConstants.BLUE_RIGHT_AUTO)),
      BlueRightPath(),
      new AutoLaunchDrive(true, X_PID, Y_PID, m_launcher, RotationPID, m_robotDrive, m_intake).withTimeout(3.5),
      new ZeroShooter(m_launcher, m_intake),
      BlueRightPathTwo(),
      new AutoLaunchDrive(true, X_PID, Y_PID, m_launcher, RotationPID, m_robotDrive, m_intake).withTimeout(3.5),
      new ZeroShooter(m_launcher, m_intake));
  }

  private Command BlueLeftAuto() {
    return new SequentialCommandGroup(
      new InstantCommand(() -> m_robotDrive.resetOdometry(AutoConstants.BLUE_LEFT_AUTO)),
      BlueLeftPath(),
      new AutoLaunchDrive(false, X_PID, Y_PID, m_launcher, RotationPID, m_robotDrive, m_intake).withTimeout(3.5),
      new ZeroShooter(m_launcher, m_intake),
      BlueLeftPathTwo(),
      new AutoLaunchDrive(false, X_PID, Y_PID, m_launcher, RotationPID, m_robotDrive, m_intake).withTimeout(3.5),
      new ZeroShooter(m_launcher, m_intake));
  }

  private Command RedRightAuto() {
    return new SequentialCommandGroup(
      new InstantCommand(() -> m_robotDrive.resetOdometry(AutoConstants.RED_RIGHT_AUTO)),
      RedRightPath(),
      new AutoLaunchDrive(true, X_PID, Y_PID, m_launcher, RotationPID, m_robotDrive, m_intake).withTimeout(3.5),
      new ZeroShooter(m_launcher, m_intake),
      RedRightPathTwo(),
      new AutoLaunchDrive(true, X_PID, Y_PID, m_launcher, RotationPID, m_robotDrive, m_intake).withTimeout(3.5),
      new ZeroShooter(m_launcher, m_intake));
  }

  private Command RedLeftAuto() {
    return new SequentialCommandGroup(
      new InstantCommand(() -> m_robotDrive.resetOdometry(AutoConstants.RED_LEFT_AUTO)),
      RedLeftPath(),
      new AutoLaunchDrive(false, X_PID, Y_PID, m_launcher, RotationPID, m_robotDrive, m_intake).withTimeout(3.5),
      new ZeroShooter(m_launcher, m_intake),
      RedLeftPathTwo(),
      new AutoLaunchDrive(false, X_PID, Y_PID, m_launcher, RotationPID, m_robotDrive, m_intake).withTimeout(3.5),
      new ZeroShooter(m_launcher, m_intake));
  }

  private Command BlueCenterAuto() {
    return new SequentialCommandGroup(
      new InstantCommand(() -> m_robotDrive.resetOdometry(AutoConstants.BLUE_CENTER_AUTO)),
      BlueCenterPath(),
      new AutoLaunchDrive(false, Center_X_PID, Center_Y_PID, m_launcher, RotationPID, m_robotDrive, m_intake).withTimeout(6),
      new ZeroShooter(m_launcher, m_intake));
  }

  private Command RedCenterAuto() {
    return new SequentialCommandGroup(
      new InstantCommand(() -> m_robotDrive.resetOdometry(AutoConstants.RED_CENTER_AUTO)),
      RedCenterPath(),
      new AutoLaunchDrive(false, Center_X_PID, Center_Y_PID, m_launcher, RotationPID, m_robotDrive, m_intake).withTimeout(6),
      new ZeroShooter(m_launcher, m_intake));
  }

  private Command BluePassingAutoRight() {
    return new SequentialCommandGroup(
      new InstantCommand(() -> m_robotDrive.resetOdometry(AutoConstants.BLUE_RIGHT_AUTO)),
      BluePassingPathRight(),
      new AutoLaunchDriveIntake(false, Center_X_PID, Center_Y_PID, m_launcher, RotationPID, m_robotDrive, m_intake)
    );
  }

  private Command BluePassingAutoLeft() {
    return new SequentialCommandGroup(
      new InstantCommand(() -> m_robotDrive.resetOdometry(AutoConstants.BLUE_LEFT_AUTO)),
      BluePassingPathLeft(),
      new AutoLaunchDriveIntake(false, Center_X_PID, Center_Y_PID, m_launcher, RotationPID, m_robotDrive, m_intake)
    );
  }

  private Command RedPassingAutoRight() {
    return new SequentialCommandGroup(
      new InstantCommand(() -> m_robotDrive.resetOdometry(AutoConstants.BLUE_LEFT_AUTO)),
      RedPassingPathRight(),
      new AutoLaunchDriveIntake(false, Center_X_PID, Center_Y_PID, m_launcher, RotationPID, m_robotDrive, m_intake)
    );
  }

  private Command RedPassingAutoLeft() {
    return new SequentialCommandGroup(
      new InstantCommand(() -> m_robotDrive.resetOdometry(AutoConstants.BLUE_LEFT_AUTO)),
      RedPassingPathLeft(),
      new AutoLaunchDriveIntake(false, Center_X_PID, Center_Y_PID, m_launcher, RotationPID, m_robotDrive, m_intake)
    );
  }

  //Auto Events
  private Command IntakeOn = new InstantCommand(() -> m_intake.setPercent(1));
  private Command IntakeOff = new InstantCommand(() -> m_intake.setPercent(0));
  private Command SpinUp = new InstantCommand(() -> m_launcher.setSpeed(3100));

  

  //Left Side Trench Auto
  Path LeftTrenchOne = new Path("Trench Path 1 - Left");
  FollowPath LTOne = (FollowPath) pathBuilder.build(LeftTrenchOne);
  /*Command LT1 = Commands.parallel(
    LTOne,
    Commands.waitUntil(() -> LTOne.getCurrentTranslationElementIndex() == 1)
        .andThen(IntakeOn),
    Commands.waitUntil(() -> LTOne.getCurrentTranslationElementIndex() == 5)
        .andThen(SpinUp)
  );*/
  Path LeftTrenchTwo = new Path("Trench Path 2 - Left");
  FollowPath LTTwo = (FollowPath) pathBuilder.build(LeftTrenchTwo);
  /*Command LT2 = Commands.parallel(
    LTTwo,
    Commands.waitUntil(() -> LTTwo.getCurrentTranslationElementIndex() == 2)
        .andThen(IntakeOn),
    Commands.waitUntil(() -> LTTwo.getCurrentTranslationElementIndex() == 6)
        .andThen(SpinUp)
  );*/

  private Command LT() {
    return new SequentialCommandGroup(
      LTOne,
      //IntakeOff,
      new AutoLaunchDrive(true, X_PID, Y_PID, m_launcher, RotationPID, m_robotDrive, m_intake).withTimeout(3.5),
      new ZeroShooter(m_launcher, m_intake),
      LTTwo,
      //IntakeOff,
      new AutoLaunchDrive(true, X_PID, Y_PID, m_launcher, RotationPID, m_robotDrive, m_intake).withTimeout(3.5),
      new ZeroShooter(m_launcher, m_intake));

    }

  //Right Side Trench Auto
  Path RightTrenchOne = new Path("Trench Path 1");
  FollowPath RTOne = (FollowPath) pathBuilder.build(RightTrenchOne);
  /*Command RT1 = Commands.parallel(
    RTOne,
    Commands.waitUntil(() -> RTOne.getCurrentTranslationElementIndex() == 1)
        .andThen(IntakeOn),
    Commands.waitUntil(() -> RTOne.getCurrentTranslationElementIndex() == 5)
        .andThen(SpinUp)
  );*/
  Path RightTrenchTwo = new Path("Trench Path 2");
  FollowPath RTTwo = (FollowPath) pathBuilder.build(RightTrenchTwo);
  /*Command RT2 = Commands.parallel(
    RTTwo,
    Commands.waitUntil(() -> RTTwo.getCurrentTranslationElementIndex() == 2)
        .andThen(IntakeOn),
    Commands.waitUntil(() -> RTTwo.getCurrentTranslationElementIndex() == 6)
        .andThen(SpinUp)
  );*/

  private Command RT() {
    return new SequentialCommandGroup(
      RTOne,
      //IntakeOff,
      new AutoLaunchDrive(true, X_PID, Y_PID, m_launcher, RotationPID, m_robotDrive, m_intake).withTimeout(3.5),
      new ZeroShooter(m_launcher, m_intake),
      RTTwo,
      //IntakeOff,
      new AutoLaunchDrive(true, X_PID, Y_PID, m_launcher, RotationPID, m_robotDrive, m_intake).withTimeout(3.5),
      new ZeroShooter(m_launcher, m_intake));

    }

      //Right Side Bump Auto
  Path RightBumpOne = new Path("Bump Path 1");
  FollowPath RBOne = (FollowPath) pathBuilder.build(RightBumpOne);
  /*Command RB1 = Commands.parallel(
    RBOne,
    Commands.waitUntil(() -> RBOne.getCurrentTranslationElementIndex() == 1)
        .andThen(IntakeOn),
    Commands.waitUntil(() -> RBOne.getCurrentTranslationElementIndex() == 4)
        .andThen(SpinUp)
  );*/
  Path RightBumpTwo = new Path("Bump Path 2");
  FollowPath RBTwo = (FollowPath) pathBuilder.build(RightBumpTwo);
  /*Command RB2 = Commands.parallel(
    RBTwo,
    Commands.waitUntil(() -> RBTwo.getCurrentTranslationElementIndex() == 3)
        .andThen(IntakeOn),
    Commands.waitUntil(() -> RBTwo.getCurrentTranslationElementIndex() == 6)
        .andThen(SpinUp)
  );*/

  private Command RB() {
    return new SequentialCommandGroup(
      RBOne,
      //IntakeOff,
      new AutoLaunchDrive(true, X_PID, Y_PID, m_launcher, RotationPID, m_robotDrive, m_intake).withTimeout(3.5),
      new ZeroShooter(m_launcher, m_intake),
      RBTwo,
      //IntakeOff,
      new AutoLaunchDrive(true, X_PID, Y_PID, m_launcher, RotationPID, m_robotDrive, m_intake).withTimeout(3.5),
      new ZeroShooter(m_launcher, m_intake));

    }

    //Left Side Bump Auto
  Path LeftBumpOne = new Path("Bump Path 1 - Left");
  FollowPath LBOne = (FollowPath) pathBuilder.build(LeftBumpOne);
  /*Command LB1 = Commands.parallel(
    LBOne,
    Commands.waitUntil(() -> LBOne.getCurrentTranslationElementIndex() == 1)
        .andThen(IntakeOn),
    Commands.waitUntil(() -> LBOne.getCurrentTranslationElementIndex() == 4)
        .andThen(SpinUp)
  );*/
  Path LeftBumpTwo = new Path("Bump Path 2 - Left");
  FollowPath LBTwo = (FollowPath) pathBuilder.build(LeftBumpTwo);
  /*Command LB2 = Commands.parallel(
    LBTwo,
    Commands.waitUntil(() -> LBTwo.getCurrentTranslationElementIndex() == 3)
        .andThen(IntakeOn),
    Commands.waitUntil(() -> LBTwo.getCurrentTranslationElementIndex() == 6)
        .andThen(SpinUp)
  );*/

  private Command LB() {
    return new SequentialCommandGroup(
      LBOne,
      //IntakeOff,
      new AutoLaunchDrive(true, X_PID, Y_PID, m_launcher, RotationPID, m_robotDrive, m_intake).withTimeout(3.5),
      new ZeroShooter(m_launcher, m_intake),
      LBTwo,
      //IntakeOff,
      new AutoLaunchDrive(true, X_PID, Y_PID, m_launcher, RotationPID, m_robotDrive, m_intake).withTimeout(3.5),
      new ZeroShooter(m_launcher, m_intake));

    }

    //Depot Bump Auto
  Path DepotIntakeB = new Path("Depot Path Intake");
  FollowPath DPIB = (FollowPath) pathBuilder.build(DepotIntakeB);
  
  Path DepotLaunchB = new Path("Depot Path Launch");
  FollowPath DPLB = (FollowPath) pathBuilder.build(DepotLaunchB);

  Path DepotBump = new Path("Depot Path Bump");
  FollowPath DPB = (FollowPath) pathBuilder.build(DepotBump);

  private Command DB() {
    return new SequentialCommandGroup(
      DPIB,
      DPLB,
      new AutoLaunchDrive(true, X_PID, Y_PID, m_launcher, RotationPID, m_robotDrive, m_intake).withTimeout(3.5),
      new ZeroShooter(m_launcher, m_intake),
      DPB,
      new AutoLaunchDrive(true, X_PID, Y_PID, m_launcher, RotationPID, m_robotDrive, m_intake).withTimeout(3.5),
      new ZeroShooter(m_launcher, m_intake));

    }

    //Depot Trench Auto
  Path DepotIntakeT = new Path("Depot Path Intake");
  FollowPath DPIT = (FollowPath) pathBuilder.build(DepotIntakeT);
  
  Path DepotLaunchT = new Path("Depot Path Launch");
  FollowPath DPLT = (FollowPath) pathBuilder.build(DepotLaunchT);
  Path DepotTrench = new Path("Depot Path Trench");
  FollowPath DPT = (FollowPath) pathBuilder.build(DepotTrench);

  private Command DT() {
    return new SequentialCommandGroup(
      DPIT,
      DPLT,
      new AutoLaunchDrive(true, X_PID, Y_PID, m_launcher, RotationPID, m_robotDrive, m_intake).withTimeout(3.5),
      new ZeroShooter(m_launcher, m_intake),
      DPT,
      new AutoLaunchDrive(true, X_PID, Y_PID, m_launcher, RotationPID, m_robotDrive, m_intake).withTimeout(3.5),
      new ZeroShooter(m_launcher, m_intake));

    }

    //Tuning Auto
  Path Tuning = new Path("Tuning Path");
  FollowPath Tune = (FollowPath) pathBuilder.build(Tuning);


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return m_chooser.getSelected();
  }

}
