package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LauncherSubsystem;

public class AutoLaunchDriveLess extends Command {
    private final DriveSubsystem m_drive;
    private final LauncherSubsystem m_launcher;
    private final double X_POSITION = 2.552;
    private final double Y_POSITION = 0.656;
    private final double FIELD_X = 16.540988;
    private final double FIELD_Y = 8.069326;
    private double m_X;
    private double m_Y;
    private final boolean m_rightSide;
    private final ProfiledPIDController m_XDrivePID;
    private final ProfiledPIDController m_YDrivePID;
    private final ProfiledPIDController m_RotationPID;
    private final IntakeSubsystem m_intake;
    
    private double intakeSetpoint;
    private boolean m_isFeeding = false; // Tracks if we are currently in a firing sequence

    public AutoLaunchDriveLess(boolean rightSide, ProfiledPIDController XDrivePID,ProfiledPIDController YDrivePID, LauncherSubsystem launcher, ProfiledPIDController PID, DriveSubsystem drive, IntakeSubsystem intake) {
        m_drive = drive;
        m_launcher = launcher;
        m_XDrivePID = XDrivePID;
        m_YDrivePID = YDrivePID;
        m_RotationPID = PID;
        m_intake = intake;
        m_rightSide = rightSide;

        addRequirements(m_drive, m_launcher, m_intake);
    }

    private boolean distanceCheck() {
        // Returns true if passing mode is on OR we are beyond the threshold distance
        return m_drive.passing || m_drive.adjustedDistance > 2.436;
    }

    @Override
    public void initialize() {
        // Reset the Profiled PID with current position AND velocity 
        // This prevents the "wrong way jerk" by matching the robot's existing momentum
        double currentAngle = m_drive.convertGyroAngle(m_drive.getAngle());
        double currentVelocity = m_drive.getTurnRate(); 
        
        m_RotationPID.reset(currentAngle, currentVelocity);
        m_XDrivePID.reset(m_drive.getPose().getX(), m_drive.getFieldRelativeSpeeds().vxMetersPerSecond);
        m_YDrivePID.reset(m_drive.getPose().getY(), m_drive.getFieldRelativeSpeeds().vyMetersPerSecond);
        
        intakeSetpoint = 0.0;
        m_isFeeding = false; // Reset firing state

        if(m_drive.redAlliance){
            if(m_rightSide){
                m_X = FIELD_X - X_POSITION;
                m_Y = Y_POSITION - Y_POSITION;
            }else{
                m_X = FIELD_X - X_POSITION;
                m_Y = FIELD_Y;
            }
        }else{
            if(m_rightSide){
                m_X = X_POSITION;
                m_Y = Y_POSITION;
            }else{
                m_X = X_POSITION;
                m_Y = FIELD_Y - Y_POSITION;
            }
        }

    }

    @Override
    public void execute() {
        // 1. Drive & Rotate
        // Passing the converted gyro directly to the PID (which has continuous input enabled)
        m_drive.drive(
                m_XDrivePID.calculate(m_drive.getPose().getX(), m_X),
                m_YDrivePID.calculate(m_drive.getPose().getY(), m_Y),
                m_RotationPID.calculate(m_drive.convertGyroAngle(m_drive.getAngle()), m_drive.rotationSetpoint),
                false);

        // 2. Update Subsystem Targets
        m_launcher.setSpeed(m_drive.adjustedRPM);
        m_intake.changeSetpoint(intakeSetpoint);

        // 3. Hysteresis Logic for Rapid Fire
        double currentRPM = m_launcher.shooterEncoder.getVelocity();
        double shooterError = Math.abs(m_launcher.targetSpeed - currentRPM);
        boolean rotationAligned = MathUtil.isNear(m_drive.rotationSetpoint, m_drive.convertGyroAngle(m_drive.getAngle()), m_drive.finalTolerance);
        
        if (!m_isFeeding) {
            // Precision Start: Only start feeding if we are within 100 RPM and aligned
            if (shooterError < 100 && rotationAligned && distanceCheck()) {
                m_isFeeding = true;
            }
        } else {
            // Aggressive Hold: Keep feeding even if RPM drops, as long as it stays within 500 RPM
            // If the robot is bumped and rotation is lost, stop feeding immediately for safety
            if (shooterError > 500 || !rotationAligned || !distanceCheck()) {
                m_isFeeding = false;
            }
        }

        // 4. Actuate Feeder and Intake
        if (m_isFeeding) {
            m_launcher.feed();
            
            // Increment intake setpoint to push balls toward the shooter
            if (!m_launcher.isJammed()) {
                intakeSetpoint += 0.006;
            }
        } else {
            m_launcher.feedOff();
            // Optional: Pull intake setpoint back slightly to prevent "pre-loading" jams
            // intakeSetpoint = MathUtil.clamp(intakeSetpoint - 0.001, 0.0, 0.25); 
        }

        intakeSetpoint = MathUtil.clamp(intakeSetpoint, 0.0, 0.25);
    }

    @Override
    public void end(boolean interrupted) {
        m_launcher.feedOff();
        m_isFeeding = false;
    }

    @Override
    public boolean isFinished() {
        return MathUtil.isNear(m_Y, m_drive.getPose().getMeasureY().magnitude(), 0.1) && MathUtil.isNear(m_X, m_drive.getPose().getMeasureX().magnitude(), 0.1);
    }
}