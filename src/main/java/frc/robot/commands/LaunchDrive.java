package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LauncherSubsystem;

public class LaunchDrive extends Command {
    double aimX;
    double aimY;
    final double BLUE_HUB_X = 4.626;
    final double RED_HUB_X = 11.915;
    final double HUB_Y = 4.035;
    final double BLUE_PASS_X = 2.313;
    final double PASS_Y_RIGHT = 2.017;
    final double PASS_Y_LEFT = 6.052;
    final double RED_PASS_X = 14.228;
    boolean redAlliance = false;
    DriveSubsystem m_drive;
    CommandXboxController m_controller;
    LauncherSubsystem m_launcher;
    ProfiledPIDController m_PID;
    IntakeSubsystem m_intake;
    Translation2d aimLocation;
    double lastTOF = 0.0;
    double adjustedRPM = 0.0;
    double finalTolerance;

    public LaunchDrive(DriveSubsystem drive, CommandXboxController controller, LauncherSubsystem launcher, ProfiledPIDController PID, IntakeSubsystem intake){
        m_drive = drive;
        m_controller = controller;
        m_launcher = launcher;
        m_PID = PID;
        m_intake = intake;

        addRequirements(m_drive, m_launcher, m_intake);
    }


    private double convertGyroAngle(double angle) {
        double result = angle % 360;
        if (result < 0) {
            result += 360;
        }
        return result;
    }
    
    private double findAngle(Translation2d position) {
        double a = (position.getX() - m_drive.getPose().getTranslation().getX());
        double b = (position.getY() - m_drive.getPose().getTranslation().getY());
        SmartDashboard.putNumber("Hub Position X", position.getX());
        SmartDashboard.putNumber("Hub Position Y", position.getY());
        SmartDashboard.putNumber("Robot Position X", m_drive.getPose().getTranslation().getX());
        SmartDashboard.putNumber("Robot Position Y", m_drive.getPose().getTranslation().getY());
        return Math.atan2(b, a);
    }

    private Translation2d getPredictedTargetPosition(Translation2d targetPos, ChassisSpeeds robotVelocity) {
        
        final double LAUNCH_ANGLE_RADS = Math.toRadians(70.0);

        Translation2d robotPos = m_drive.getPose().getTranslation();
        Translation2d predictedPos = targetPos;
        double timeOfFlight = 0.0;
        double newRPM = 0.0;

        for (int i = 0; i < 3; i++) {
            double distance = robotPos.getDistance(predictedPos);
            double radialVel = m_drive.getVelocityFromTarget(aimLocation, m_drive.getFieldRelativeSpeeds());
            double effectiveDistance = distance - (radialVel * timeOfFlight);
            newRPM = m_launcher.getRPMForDistance(effectiveDistance);
            double horizontalVel = (newRPM * 4 * Math.PI / 60 / 12 / 2.222) * Math.cos(LAUNCH_ANGLE_RADS);
            double totalVel = horizontalVel + radialVel;
            timeOfFlight = (distance / totalVel);
            predictedPos = new Translation2d(
                targetPos.getX() + (-robotVelocity.vxMetersPerSecond * timeOfFlight),
                targetPos.getY() + (-robotVelocity.vyMetersPerSecond * timeOfFlight)
            );
        }
        this.adjustedRPM = newRPM;
        this.lastTOF = timeOfFlight;
        return predictedPos;
    }

    @Override
    public void initialize() {
        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent()) {
            redAlliance = alliance.get() == DriverStation.Alliance.Red;
        }
    }

    @Override
    public void execute(){
        if(!redAlliance) {
            if(m_drive.getPose().getX() < BLUE_HUB_X) {
                aimX = BLUE_HUB_X;
                aimY = HUB_Y;
            }else {
                aimX = BLUE_PASS_X;
                if(m_drive.getPose().getY() < HUB_Y) {
                    aimY = PASS_Y_RIGHT;
                } else {
                    aimY = PASS_Y_LEFT;
                }
            }
        }else {
            if(m_drive.getPose().getX() > RED_HUB_X) {
                aimX = RED_HUB_X;
                aimY = HUB_Y;
            }else {
                aimX = RED_PASS_X;
                if(m_drive.getPose().getY() < HUB_Y) {
                    aimY = PASS_Y_RIGHT;
                }else {
                    aimY = PASS_Y_LEFT;
                }
            }
        }
        aimLocation = new Translation2d(aimX, aimY);
        m_drive.rotationSetpoint = convertGyroAngle(Math.toDegrees(findAngle(getPredictedTargetPosition(aimLocation, m_drive.getFieldRelativeSpeeds()))));
        double currentGyro = convertGyroAngle(m_drive.getAngle());
        SmartDashboard.putNumber("currentGyro",currentGyro);
        
        if(m_drive.getPose().getX() < BLUE_HUB_X || m_drive.getPose().getX() > RED_HUB_X) {
            double distanceMeters = m_drive.getPose().getTranslation().getDistance(aimLocation);
            double slope = (2.5 - 10.0) / (5.0 - 1.0); // results in -1.875 deg/meter
            double dynamicTolerance = 10.0 + (slope * (distanceMeters - 1.0));
            finalTolerance = MathUtil.clamp(dynamicTolerance, 2.5, 10.0);
        }else {
            finalTolerance = 10;
        }

        m_drive.drive(
                -MathUtil.applyDeadband(m_controller.getRawAxis(1), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_controller.getRawAxis(0), OIConstants.kDriveDeadband),
                m_PID.calculate(currentGyro, m_drive.rotationSetpoint),
                true);
        
        m_launcher.setSpeed(adjustedRPM);

        m_intake.changeSetpoint(.13);
        
        if (MathUtil.isNear(m_launcher.targetSpeed, m_launcher.shooterEncoder.getVelocity(), 200) && MathUtil.isNear(m_drive.rotationSetpoint, convertGyroAngle(m_drive.getAngle()), finalTolerance)){
            m_launcher.feed();
        } else {
            m_launcher.feedOff();
        }
    }

    @Override
    public boolean isFinished(){
        return false;
    }
}
