// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.hal.FRCNetComm.tInstances;
import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.ADIS16470_IMU.IMUAxis;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Constants.AutoConfig;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.Launcher;
import gg.questnav.questnav.PoseFrame;
import gg.questnav.questnav.QuestNav;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveSubsystem extends SubsystemBase {
  // Create MAXSwerveModules
  private final MAXSwerveModule m_frontLeft = new MAXSwerveModule(
      DriveConstants.kFrontLeftDrivingCanId,
      DriveConstants.kFrontLeftTurningCanId,
      DriveConstants.kFrontLeftChassisAngularOffset);

  private final MAXSwerveModule m_frontRight = new MAXSwerveModule(
      DriveConstants.kFrontRightDrivingCanId,
      DriveConstants.kFrontRightTurningCanId,
      DriveConstants.kFrontRightChassisAngularOffset);

  private final MAXSwerveModule m_rearLeft = new MAXSwerveModule(
      DriveConstants.kRearLeftDrivingCanId,
      DriveConstants.kRearLeftTurningCanId,
      DriveConstants.kBackLeftChassisAngularOffset);

  private final MAXSwerveModule m_rearRight = new MAXSwerveModule(
      DriveConstants.kRearRightDrivingCanId,
      DriveConstants.kRearRightTurningCanId,
      DriveConstants.kBackRightChassisAngularOffset);

  // The gyro sensor
  private final ADIS16470_IMU m_gyro = new ADIS16470_IMU();
  public double aimX;
  public double aimY;
  final double BLUE_HUB_X = 4.626;
  final double RED_HUB_X = 11.915;
  final double HUB_Y = 4.035;
  final double BLUE_PASS_X = 3.813;
  final double PASS_Y_RIGHT = 2.017;
  final double PASS_Y_LEFT = 6.052;
  final double RED_PASS_X = 12.728;
  public double rotationSetpoint = 0;
  public boolean redAlliance = false;
  public Translation2d aimLocation;
  public double lastTOF = 0.0;
  public double adjustedRPM = 0.0;
  public double finalTolerance;
  public double adjustedDistance = 0.0;
  public double currentGyro = 0.0;
  public double hopperFill;
  private LED m_led = new LED();
  public boolean passing = false;

  private final QuestNav m_questNav = new QuestNav();

  private final StructArrayPublisher<Pose3d> m_allPosesPub;
  private final StructArrayPublisher<Pose3d> m_acceptedPosesPub;
  private final StructArrayPublisher<Pose3d> m_rejectedPosesPub;
  private final StructPublisher<Pose3d> m_latestPosePub;

  private double m_lastPoseTimestamp = -1;


  // Odometry class for tracking robot pose
  public final Field2d m_pose = new Field2d();
  public Vision vision = new Vision();
  public Pose2d initialPosition = new Pose2d();
  SwerveDrivePoseEstimator m_odometry = new SwerveDrivePoseEstimator(
      DriveConstants.kDriveKinematics,
      Rotation2d.fromDegrees(m_gyro.getAngle(IMUAxis.kZ)),
      new SwerveModulePosition[] {
          m_frontLeft.getPosition(),
          m_frontRight.getPosition(),
          m_rearLeft.getPosition(),
          m_rearRight.getPosition()
      },initialPosition);

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {
    // Usage reporting for MAXSwerve template
    HAL.report(tResourceType.kResourceType_RobotDrive, tInstances.kRobotDriveSwerve_MaxSwerve);
    //AutoBuilder for PathPlanner
    AutoBuilder.configure(
    this::getPose, // Robot pose supplier
    this::resetOdometry, // Method to reset odometry (will be called if your auto has a starting pose)
    this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
    (speeds, feedforwards) -> driveRobotRelative(speeds), // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
    new PPHolonomicDriveController( // HolonomicPathFollowerConfig, this should likely live in your Constants class
        new PIDConstants(5, 0.0, 0.0), // Translation PID constants
        new PIDConstants(5, 0, 0.0) // Rotation PID constants
    ),
    AutoConfig.config,
    () -> {
      var alliance = DriverStation.getAlliance();
      if(alliance.isPresent()) {
        return alliance.get() == DriverStation.Alliance.Red;
      }
      return false;
    },
    this // Reference to this subsystem to set requirements
  );
  
    var nt = NetworkTableInstance.getDefault();
    m_allPosesPub =
        nt.getStructArrayTopic("QuestNav/RobotPoses", Pose3d.struct).publish();
    m_acceptedPosesPub =
        nt.getStructArrayTopic("QuestNav/RobotPosesAccepted", Pose3d.struct).publish();
    m_rejectedPosesPub =
        nt.getStructArrayTopic("QuestNav/RobotPosesRejected", Pose3d.struct).publish();
    m_latestPosePub =
        nt.getStructTopic("QuestNav/LatestRobotPose", Pose3d.struct).publish();

    m_questNav.setVersionCheckEnabled(false);

    m_questNav.onConnected(() ->
        System.out.println("Quest connected!"));
    m_questNav.onDisconnected(() ->
        DriverStation.reportWarning("Quest disconnected!", false));
    m_questNav.onTrackingAcquired(() ->
        System.out.println("Quest tracking acquired!"));
    m_questNav.onTrackingLost(() ->
        DriverStation.reportWarning("Quest tracking lost!", false));
    m_questNav.onLowBattery((int) Constants.Vision.BATTERY_LOW_PERCENT, level ->
        DriverStation.reportWarning("Quest battery low: " + level + "%", false));
    m_questNav.onCommandSuccess(response ->
        System.out.println("Pose reset succeeded: " + response.getCommandId()));
    m_questNav.onCommandFailure(response ->
        DriverStation.reportError(
            "Pose reset failed: " + response.getErrorMessage(), false));
  }

  @Override
  public void periodic() {
    // Update the odometry in the periodic block

    m_pose.setRobotPose(m_odometry.getEstimatedPosition());

    SmartDashboard.putData("pose", m_pose);

    m_odometry.update(
        Rotation2d.fromDegrees(m_gyro.getAngle(IMUAxis.kZ)),
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
        });
    var visionBottomEst = vision.getFrontEstimatedGlobalPose();
    visionBottomEst.ifPresent(
      est -> {
        var estStdDevs = vision.getEstimationStdDevs();
        m_odometry.addVisionMeasurement(est.estimatedPose.toPose2d(), est.timestampSeconds, estStdDevs);
    });

    var visionTopEst = vision.getBackEstimatedGlobalPose();
    visionTopEst.ifPresent(
      est -> {
        var estStdDevs = vision.getEstimationStdDevs();
        m_odometry.addVisionMeasurement(est.estimatedPose.toPose2d(), est.timestampSeconds, estStdDevs);
    });

    m_questNav.commandPeriodic();

    // Publish device diagnostics
    boolean connected = m_questNav.isConnected();
    boolean tracking = m_questNav.isTracking();
    SmartDashboard.putBoolean("QuestNav/Connected", connected);
    SmartDashboard.putBoolean("QuestNav/Tracking", tracking);
    SmartDashboard.putNumber("QuestNav/Latency", m_questNav.getLatency());
    m_questNav.getBatteryPercent().ifPresent(b -> {
      SmartDashboard.putNumber("QuestNav/Battery%", b);
      if (b < Constants.Vision.BATTERY_CRITICAL_PERCENT) {
        DriverStation.reportWarning("Quest battery CRITICAL: " + b + "%", false);
      }
    });
    m_questNav.getTrackingLostCounter().ifPresent(
        c -> SmartDashboard.putNumber("QuestNav/TrackingLostCount", c));

    // Process all unread pose frames
    PoseFrame[] frames = m_questNav.getAllUnreadPoseFrames();
    SmartDashboard.putNumber("QuestNav/UnreadFrames", frames.length);

    List<Pose3d> allPoses = new ArrayList<>();
    List<Pose3d> acceptedPoses = new ArrayList<>();
    List<Pose3d> rejectedPoses = new ArrayList<>();

    for (PoseFrame frame : frames) {
      Pose3d questPose = frame.questPose3d();
      Pose3d robotPose = questPose.plus(Constants.Vision.ROBOT_TO_QUEST.inverse());

      allPoses.add(robotPose);

      if (shouldReject(robotPose)) {
        rejectedPoses.add(robotPose);
        continue;
      }

      acceptedPoses.add(robotPose);

      if (frame.isTracking()) {
        m_odometry.addVisionMeasurement(
            robotPose.toPose2d(), frame.dataTimestamp(), Constants.Vision.QUESTNAV_STD_DEVS);
      }

      m_lastPoseTimestamp = frame.dataTimestamp();
    }

    // Publish pose arrays for AdvantageScope 3D field visualization
    m_allPosesPub.set(allPoses.toArray(Pose3d[]::new));
    m_acceptedPosesPub.set(acceptedPoses.toArray(Pose3d[]::new));
    m_rejectedPosesPub.set(rejectedPoses.toArray(Pose3d[]::new));

    if (!allPoses.isEmpty()) {
      m_latestPosePub.set(allPoses.get(allPoses.size() - 1));
    }

    if (m_lastPoseTimestamp > 0) {
      SmartDashboard.putNumber(
          "QuestNav/TimeSinceLastPose", Timer.getTimestamp() - m_lastPoseTimestamp);
    }

    //Aiming Stuff

    if(!redAlliance()) {
            if(getPose().getX() < BLUE_HUB_X) {
                aimX = BLUE_HUB_X;
                aimY = HUB_Y;
                passing = false;
                if (adjustedDistance > 2.436 && !warningLight()){
                  m_led.SolidGreen();
                }else if (adjustedDistance > 2.436 && warningLight()){
                  m_led.BlinkGreen();
                }else if (adjustedDistance <= 2.436 && !warningLight()){
                  m_led.SolidRed();
                }else if (adjustedDistance <= 2.436 && warningLight()){
                  m_led.BlinkRed();
                }
            }else {
                aimX = BLUE_PASS_X;
                passing = true;
                if (!warningLight()){
                  m_led.SolidBlue();
                }else if (warningLight()){
                  m_led.BlinkBlue();
                }
                if(getPose().getY() < HUB_Y) {
                    aimY = PASS_Y_RIGHT;
                } else {
                    aimY = PASS_Y_LEFT;
                }
            }
        }else {
            if(getPose().getX() > RED_HUB_X) {
                aimX = RED_HUB_X;
                aimY = HUB_Y;
                passing = false;
                if (adjustedDistance > 2.436 && !warningLight()){
                  m_led.SolidGreen();
                }else if (adjustedDistance > 2.436 && warningLight()){
                  m_led.BlinkGreen();
                }else if (adjustedDistance <= 2.436 && !warningLight()){
                  m_led.SolidRed();
                }else if (adjustedDistance <= 2.436 && warningLight()){
                  m_led.BlinkRed();
                }
            }else {
                aimX = RED_PASS_X;
                passing = true;
                if (!warningLight()){
                  m_led.SolidBlue();
                }else if (warningLight()){
                  m_led.BlinkBlue();
                }
                if(getPose().getY() < HUB_Y) {
                    aimY = PASS_Y_RIGHT;
                }else {
                    aimY = PASS_Y_LEFT;
                }
            }
        }
        aimLocation = new Translation2d(aimX, aimY);
        rotationSetpoint = convertGyroAngle(Math.toDegrees(findAngle(getPredictedTargetPosition(aimLocation, getFieldRelativeSpeeds()))));
        currentGyro = convertGyroAngle(getAngle());

        
        if(getPose().getX() < BLUE_HUB_X || getPose().getX() > RED_HUB_X) {
            double distanceMeters = getPose().getTranslation().getDistance(aimLocation);
            double slope = (2.5 - 5.0) / (5.0 - 2.436);
            double dynamicTolerance = 5.0 + (slope * (distanceMeters - 2.436));
            finalTolerance = MathUtil.clamp(dynamicTolerance, 2.5, 5.0);
        }else {
            finalTolerance = 5;
        }

  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return m_odometry.getEstimatedPosition();
  }
 

  public double getAngle() {
    return m_odometry.getEstimatedPosition().getRotation().getDegrees();
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    m_odometry.resetPosition(
        Rotation2d.fromDegrees(m_gyro.getAngle(IMUAxis.kZ)),
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
        },
        pose);
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed        Speed of the robot in the x direction (forward).
   * @param ySpeed        Speed of the robot in the y direction (sideways).
   * @param rot           Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the
   *                      field.
   */
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
      if(fieldRelative){
        var alliance = DriverStation.getAlliance();
        boolean red = false;
        if(alliance.isPresent()) {
          red = alliance.get() == DriverStation.Alliance.Red;
        }
        if(red){
          xSpeed = -xSpeed;
          ySpeed = -ySpeed;
        }

      }
    // Convert the commanded speeds into the correct units for the drivetrain
    double xSpeedDelivered = xSpeed * DriveConstants.kMaxSpeedMetersPerSecond;
    double ySpeedDelivered = ySpeed * DriveConstants.kMaxSpeedMetersPerSecond;
    double rotDelivered = rot * DriveConstants.kMaxAngularSpeed;

    var swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(
        fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered,
                Rotation2d.fromDegrees(getAngle()))
            : new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered));
    SwerveDriveKinematics.desaturateWheelSpeeds(
        swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_rearLeft.setDesiredState(swerveModuleStates[2]);
    m_rearRight.setDesiredState(swerveModuleStates[3]);
  }

  /**
   * Sets the wheels into an X formation to prevent movement.
   */
  public void setX() {
    m_frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    m_frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_rearLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_rearRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
  }

  /**
   * Sets the swerve ModuleStates.
   *
   * @param desiredStates The desired SwerveModule states.
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(
        desiredStates, DriveConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(desiredStates[0]);
    m_frontRight.setDesiredState(desiredStates[1]);
    m_rearLeft.setDesiredState(desiredStates[2]);
    m_rearRight.setDesiredState(desiredStates[3]);
  }

  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {
    m_frontLeft.resetEncoders();
    m_rearLeft.resetEncoders();
    m_frontRight.resetEncoders();
    m_rearRight.resetEncoders();
  }

  /** Zeroes the heading of the robot. */
  public void zeroHeading() {
    m_gyro.reset();
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public double getHeading() {
    return Rotation2d.fromDegrees(m_gyro.getAngle(IMUAxis.kZ)).getDegrees();
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return m_gyro.getRate(IMUAxis.kZ) * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
  }

   /**Gives Swerve module states for use in PathPlanner */
  public SwerveModuleState[] getStates() {
    return new SwerveModuleState[] {
      m_frontLeft.getState(),
      m_frontRight.getState(),
      m_rearLeft.getState(),
      m_rearRight.getState()
    };
  }

  /**Returns Robot Relative Speeds for use in PathPlanner */
  public ChassisSpeeds getRobotRelativeSpeeds(){
    return DriveConstants.kDriveKinematics.toChassisSpeeds(getStates());
  }

  public ChassisSpeeds getFieldRelativeSpeeds(){
    ChassisSpeeds robotSpeeds = getRobotRelativeSpeeds();
    return ChassisSpeeds.fromRobotRelativeSpeeds(
      robotSpeeds.vxMetersPerSecond,
      robotSpeeds.vyMetersPerSecond,
      robotSpeeds.omegaRadiansPerSecond,
      Rotation2d.fromDegrees(getAngle())
    );
  }
  
    /**Used to drive the robot in robot relative orientation for use in PathPlanner */
  public void driveRobotRelative(ChassisSpeeds speeds) {
    var targetStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(speeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(targetStates, DriveConstants.kMaxSpeedMetersPerSecond);
    setModuleStates(targetStates);
  }



  public double getVelocityFromTarget(Translation2d targetFieldPosition, ChassisSpeeds fieldRelativeSpeeds) {
    Translation2d robotFieldPosition = this.getPose().getTranslation();
    Translation2d delta = targetFieldPosition.minus(robotFieldPosition);
    double distance = delta.getNorm();

    if (distance < 0.1) return 0.0;

    double unitX = delta.getX() / distance;
    double unitY = delta.getY() / distance;

    return (fieldRelativeSpeeds.vxMetersPerSecond * unitX) + (fieldRelativeSpeeds.vyMetersPerSecond * unitY);
  }

  //Aiming Stuff

      public double convertGyroAngle(double angle) {
        double result = angle % 360;
        if (result < 0) {
            result += 360;
        }
        return result;
    }
    
    private double findAngle(Translation2d position) {
        double a = (position.getX() - getPose().getTranslation().getX());
        double b = (position.getY() - getPose().getTranslation().getY());

        return Math.atan2(b, a);
    }

    private Translation2d getPredictedTargetPosition(Translation2d targetPos, ChassisSpeeds robotVelocity) {
        
        final double LAUNCH_ANGLE_RADS = Math.toRadians(65.0);

        Translation2d robotPos = getPose().getTranslation();
        Translation2d predictedPos = targetPos;
        double timeOfFlight = lastTOF;
        double newRPM = adjustedRPM;
        double effectiveDistance = adjustedDistance;

        for (int i = 0; i < 5; i++) {
            double distance = robotPos.getDistance(predictedPos);
            double radialVel = getVelocityFromTarget(aimLocation, getFieldRelativeSpeeds());
            effectiveDistance = distance - (radialVel * timeOfFlight);
            newRPM = Launcher.rpmTable.get(effectiveDistance);
            double horizontalVel = (newRPM * 4 * Math.PI * 0.3048 / 60 / 12 / 2.222) * Math.cos(LAUNCH_ANGLE_RADS);
            double totalVel = horizontalVel - radialVel;
            timeOfFlight = (distance / totalVel);
            predictedPos = new Translation2d(
                targetPos.getX() - (robotVelocity.vxMetersPerSecond * timeOfFlight),
                targetPos.getY() - (robotVelocity.vyMetersPerSecond * timeOfFlight)
            );
            
        }
        this.adjustedRPM = newRPM;
        this.lastTOF = timeOfFlight;
        this.adjustedDistance = effectiveDistance;
        return predictedPos;
    }


    public boolean redAlliance() {
        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent()) {
            redAlliance = alliance.get() == DriverStation.Alliance.Red;
        }
        return redAlliance;
    }

    public boolean warningLight(){
      boolean light = false;
      double matchTime =DriverStation.getMatchTime();
      if((matchTime <= 135 && matchTime > 130) || // Before Shift 1
      (matchTime <= 112 && matchTime > 105) || // Before Shift 2
      (matchTime <= 87  && matchTime > 80)  || // Before Shift 3
      (matchTime <= 62  && matchTime > 55)  || // Before Shift 4
      (matchTime <= 37  && matchTime > 30)  ||
      (matchTime <= 7   && matchTime > 0))    // Before Endgame
      {
        light = true;
      }else{
        light = false;
      }
      return light;
      }

  private boolean shouldReject(Pose3d pose) {
    return pose.getX() < 0.0
        || pose.getX() > Constants.Vision.kTagLayout.getFieldLength()
        || pose.getY() < 0.0
        || pose.getY() > Constants.Vision.kTagLayout.getFieldWidth();
  }

  public void resetPose(Pose3d robotPose) {
    Pose3d questPose = robotPose.plus(Constants.Vision.ROBOT_TO_QUEST);
    m_questNav.setPose(questPose);
  }

  public void resetPose(Pose2d robotPose) {
    resetPose(new Pose3d(robotPose));
  }

  }
