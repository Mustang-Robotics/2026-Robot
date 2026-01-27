package frc.robot.commands;

import edu.wpi.first.math.MathUtil;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.DriveSubsystem;

public class RobotCentricDrive extends Command{
    DriveSubsystem m_drive;
    CommandXboxController m_controller;

    public RobotCentricDrive(DriveSubsystem drive, CommandXboxController controller){
        m_drive = drive;
        m_controller = controller;

        addRequirements(m_drive);
    }

    @Override
    public void initialize() {
        // Ensure odometry reflects the last saved pose when switching modes
        m_drive.restorePose();
    }

    @Override
    public void execute(){
        m_drive.drive(
                    -MathUtil.applyDeadband(m_controller.getRawAxis(1), OIConstants.kDriveDeadband),
                    -MathUtil.applyDeadband(m_controller.getRawAxis(0), OIConstants.kDriveDeadband),
                    -MathUtil.applyDeadband(m_controller.getRawAxis(4), OIConstants.kDriveDeadband),
                false);
    }

    @Override
    public boolean isFinished(){
        return false;
    }
}
