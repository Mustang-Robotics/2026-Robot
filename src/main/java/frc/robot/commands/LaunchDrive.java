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
    double x;
    double y = 4.035;
    DriveSubsystem m_drive;
    CommandXboxController m_controller;
    LauncherSubsystem m_launcher;
    ProfiledPIDController m_PID;
    IntakeSubsystem m_intake;
    Translation2d Hub;
    double lastTOF = 0.0;
    double adjustedRPM = 0.0;

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

        for (int i = 0; i < 5; i++) {
            double distance = robotPos.getDistance(predictedPos);
            double radialVel = m_drive.getVelocityFromTarget(Hub, m_drive.getFieldRelativeSpeeds());
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
        boolean red = false;
        if (alliance.isPresent()) {
            red = alliance.get() == DriverStation.Alliance.Red;
        }
        if (red) {
            x = 11.8;
        }else {
            x = 4.626;
        }

        Hub = new Translation2d(x, y);
    }

    @Override
    public void execute(){
        m_drive.rotationSetpoint = convertGyroAngle(Math.toDegrees(findAngle(getPredictedTargetPosition(Hub, m_drive.getFieldRelativeSpeeds()))));
        double currentGyro = convertGyroAngle(m_drive.getAngle());
        SmartDashboard.putNumber("currentGyro",currentGyro);
        m_drive.drive(
                -MathUtil.applyDeadband(m_controller.getRawAxis(1), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_controller.getRawAxis(0), OIConstants.kDriveDeadband),
                m_PID.calculate(currentGyro, m_drive.rotationSetpoint),
                true);
        
        m_launcher.setSpeed(adjustedRPM);

        m_intake.changeSetpoint(.13);
        
        if (MathUtil.isNear(m_launcher.targetSpeed, m_launcher.shooterEncoder.getVelocity(), 200) && MathUtil.isNear(m_drive.rotationSetpoint, convertGyroAngle(m_drive.getAngle()), 5)){
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
