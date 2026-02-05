package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LauncherSubsystem;

public class LaunchDrive extends Command {
    double x;
    double y = 4.035;
    DriveSubsystem m_drive;
    double distance;
    CommandXboxController m_controller;
    LauncherSubsystem m_launcher;
    PIDController m_PID;
    Translation2d Hub;
    double timeOfFlight;

    public LaunchDrive(DriveSubsystem drive, CommandXboxController controller, LauncherSubsystem launcher, PIDController PID){
        m_drive = drive;
        m_controller = controller;
        m_launcher = launcher;
        m_PID = PID;

        addRequirements(m_drive, m_launcher);
    }


    private double convertGyroAngle(double angle) {
        double result = angle % 360;
        if (result < 0) {
            result += 360;
        }
        return result;
    }

    private double findAngle(double x, double y) {
        double a = (x - m_drive.getPose().getMeasureY().baseUnitMagnitude());
        double b = (y - m_drive.getPose().getMeasureX().baseUnitMagnitude());
        return Math.atan2(a, b);
    }
    
    private double findAngle(Translation2d position) {
        double a = (position.getX() - m_drive.getPose().getMeasureY().baseUnitMagnitude());
        double b = (position.getY() - m_drive.getPose().getMeasureX().baseUnitMagnitude());
        return Math.atan2(a, b);
    }

    private Translation2d getPredictedTargetPosition(Translation2d targetPos, ChassisSpeeds targetVelocity, double launcherRPM) {
        
        final double LAUNCH_ANGLE_RADS = Math.toRadians(70.0);

        double horizontalSpeed = (launcherRPM * 4 * Math.PI / 60 / 12 / 2.222) * Math.cos(LAUNCH_ANGLE_RADS); // Convert RPM to m/s

        Translation2d robotPos = m_drive.getPose().getTranslation();
        Translation2d predictedPos = targetPos;
        timeOfFlight = 0.0;

        for (int i = 0; i < 5; i++) {
            distance = robotPos.getDistance(predictedPos);
            timeOfFlight = (distance / horizontalSpeed) + 0.02;
            predictedPos = new Translation2d(
                targetPos.getX() + (targetVelocity.vxMetersPerSecond * timeOfFlight),
                targetPos.getY() + (targetVelocity.vyMetersPerSecond * timeOfFlight)
            );
        }

        return predictedPos;
    }

    private double getAdjustedRPM() {
        double actualDistance = distance;
        double radialVel;
        double effectiveDistance = actualDistance- (radialVel * timeOfFlight);
        return m_launcher.getRPMForDistance(effectiveDistance);
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
        m_drive.rotationSetpoint = convertGyroAngle(Math.toDegrees(findAngle(getPredictedTargetPosition(Hub, m_drive.getFieldRelativeSpeeds(), m_launcher.getRPMForDistance(distance)))));
        m_drive.drive(
                -MathUtil.applyDeadband(m_controller.getRawAxis(1), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_controller.getRawAxis(0), OIConstants.kDriveDeadband),
                m_PID.calculate(convertGyroAngle(m_drive.getAngle()), m_drive.rotationSetpoint),
                true);
        
        m_launcher.setSpeed(m_launcher.getRPMForDistance(distance));
        
        if (MathUtil.isNear(m_launcher.targetSpeed, m_launcher.shooterEncoder.getVelocity(), 200) && angleReady){
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
