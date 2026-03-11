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


    public LaunchDrive(DriveSubsystem drive, CommandXboxController controller, LauncherSubsystem launcher, ProfiledPIDController PID, IntakeSubsystem intake){
        m_drive = drive;
        m_controller = controller;
        m_launcher = launcher;
        m_PID = PID;
        m_intake = intake;

        addRequirements(m_drive, m_launcher, m_intake);
    }




    @Override
    public void execute(){

        m_drive.drive(
                -MathUtil.applyDeadband(m_controller.getRawAxis(1), OIConstants.kDriveDeadband)*.3,
                -MathUtil.applyDeadband(m_controller.getRawAxis(0), OIConstants.kDriveDeadband)*.3,
                m_PID.calculate(m_drive.currentGyro, m_drive.rotationSetpoint),
                true);
        
        m_launcher.setSpeed(m_drive.adjustedRPM);

        m_intake.changeSetpoint(.13);

        if (MathUtil.isNear(m_launcher.targetSpeed, m_launcher.shooterEncoder.getVelocity(), 200) && MathUtil.isNear(m_drive.rotationSetpoint, m_drive.convertGyroAngle(m_drive.getAngle()), m_drive.finalTolerance) && m_drive.adjustedDistance > 2.436){
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
