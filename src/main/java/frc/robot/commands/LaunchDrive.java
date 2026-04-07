package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LauncherSubsystem;

public class LaunchDrive extends Command {
    DriveSubsystem m_drive;
    CommandXboxController m_controller;
    LauncherSubsystem m_launcher;
    ProfiledPIDController m_PID;
    IntakeSubsystem m_intake;
    double intakeSetpoint;

    public LaunchDrive(DriveSubsystem drive, CommandXboxController controller, LauncherSubsystem launcher, ProfiledPIDController PID, IntakeSubsystem intake) {
        m_drive = drive;
        m_controller = controller;
        m_launcher = launcher;
        m_PID = PID;
        m_intake = intake;

        addRequirements(m_drive, m_launcher, m_intake);
    }

    private boolean distanceCheck(){
        boolean launch;

        if(m_drive.passing){
            launch = true;
        }else if(m_drive.adjustedDistance > 2.436){
            launch = true;
        }else {
            launch = false;
        }
        return launch;
    }

    @Override
    public void initialize() {
 
        double currentAngle = m_drive.currentGyro;
        double currentVelocity = m_drive.getTurnRate(); 
        m_PID.reset(currentAngle, currentVelocity);
        

        intakeSetpoint = 0.0;

    }

    @Override
    public void execute() {
        m_drive.drive(
                -MathUtil.applyDeadband(m_controller.getRawAxis(1), OIConstants.kDriveDeadband) * 0.3,
                -MathUtil.applyDeadband(m_controller.getRawAxis(0), OIConstants.kDriveDeadband) * 0.3,
                m_PID.calculate(m_drive.currentGyro, m_drive.rotationSetpoint),
                true);

        m_launcher.setSpeed(m_drive.adjustedRPM);

        m_intake.changeSetpoint(intakeSetpoint);

        // Check conditions: RPM matched, Rotation aligned, and Distance is valid
        boolean isReady = MathUtil.isNear(m_launcher.targetSpeed, m_launcher.shooterEncoder.getVelocity(), 200) 
                       && MathUtil.isNear(m_drive.rotationSetpoint, m_drive.convertGyroAngle(m_drive.getAngle()), m_drive.finalTolerance) 
                       && distanceCheck();

        if (isReady) {
            m_launcher.feed();
            
            // Pause intake setpoint growth if we are clearing a jam (auto or manual)
            if (!m_launcher.isJammed()) {
                intakeSetpoint += 0.0015;
            }
        } else {
            m_launcher.feedOff();
        }

        intakeSetpoint = MathUtil.clamp(intakeSetpoint, 0.0, 0.25);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
