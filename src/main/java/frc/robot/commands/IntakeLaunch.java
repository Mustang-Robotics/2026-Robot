package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LauncherSubsystem;

public class IntakeLaunch extends Command {
    private final DriveSubsystem m_drive;
    private final CommandXboxController m_controller;
    private final LauncherSubsystem m_launcher;
    private final ProfiledPIDController m_PID;
    private final IntakeSubsystem m_intake;
    
    private double intakeSetpoint;
    private boolean m_isFeeding = false; // NEW: Hysteresis state

    public IntakeLaunch(DriveSubsystem drive, CommandXboxController controller, LauncherSubsystem launcher, ProfiledPIDController PID, IntakeSubsystem intake){
        m_drive = drive;
        m_controller = controller;
        m_launcher = launcher;
        m_PID = PID;
        m_intake = intake;

        addRequirements(m_drive, m_launcher, m_intake);
    }

    @Override
    public void initialize(){
        // Smooth Handoff: Reset PID with both position and current spin velocity
        double currentAngle = m_drive.convertGyroAngle(m_drive.getAngle());
        double currentVelocity = m_drive.getTurnRate(); 
        m_PID.reset(currentAngle, currentVelocity);
        
        intakeSetpoint = 0.005;
        m_intake.setPercent(1);
        m_isFeeding = false; // Reset state
    }

    @Override
    public void execute(){
        // 1. Driving with smooth Rotation PID
        m_drive.drive(
                -MathUtil.applyDeadband(m_controller.getRawAxis(1), OIConstants.kDriveDeadband)*.3,
                -MathUtil.applyDeadband(m_controller.getRawAxis(0), OIConstants.kDriveDeadband)*.3,
                m_PID.calculate(m_drive.convertGyroAngle(m_drive.getAngle()), m_drive.rotationSetpoint),
                true);
        
        m_launcher.setSpeed(m_drive.adjustedRPM);
        m_intake.changeSetpoint(intakeSetpoint);

        // 2. Performance Checks
        double shooterError = Math.abs(m_launcher.targetSpeed - m_launcher.shooterEncoder.getVelocity());
        boolean rotationAligned = MathUtil.isNear(m_drive.rotationSetpoint, m_drive.convertGyroAngle(m_drive.getAngle()), m_drive.finalTolerance);
        boolean distanceValid = m_drive.adjustedDistance > 2.436;

        // 3. Hysteresis Logic (100 RPM Start / 500 RPM Stay-Alive)
        if (!m_isFeeding) {
            // Must be precise to start the first shot
            if (shooterError < 100 && rotationAligned && distanceValid) {
                m_isFeeding = true;
            }
        } else {
            // Stay aggressive: keep feeding unless the drop is catastrophic
            if (shooterError > 500 || !rotationAligned || !distanceValid) {
                m_isFeeding = false;
            }
        }

        // 4. Actuation
        if (m_isFeeding) {
            m_launcher.feed();
        } else {
            m_launcher.feedOff();
        }
    }

    @Override
    public void end(boolean interrupted) {
        m_launcher.feedOff();
        m_isFeeding = false;
    }

    @Override
    public boolean isFinished(){
        return false;
    }
}