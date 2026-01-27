package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.DriveSubsystem;

public class IntakeDrive extends Command{
    DriveSubsystem m_drive;
    CommandXboxController m_controller;
    PIDController m_PID;

    public IntakeDrive(DriveSubsystem drive, CommandXboxController controller, PIDController PID){
        m_drive = drive;
        m_controller = controller;
        m_PID = PID;

        addRequirements(m_drive);
    }

    @Override
    public void initialize() {
        // Restore odometry to the saved pose when switching into intake-aiming drive
        m_drive.restorePose();
    }

    private double convertAngle(double angle) {
        double result = angle % 360;
        if (result < 0) {
            result += 360;
        }
        return result;
    }

    private double driveStickAngle(double x, double y) {
        double result = Math.toDegrees(Math.atan2(y,x));
        return result;
    }

    @Override
    public void execute(){
        m_drive.intakeSetpoint = convertAngle(driveStickAngle(MathUtil.applyDeadband(m_controller.getRawAxis(1), OIConstants.kDriveDeadband), MathUtil.applyDeadband(m_controller.getRawAxis(0), OIConstants.kDriveDeadband)));

        m_drive.drive(
                -MathUtil.applyDeadband(m_controller.getRawAxis(1), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_controller.getRawAxis(0), OIConstants.kDriveDeadband),
                m_PID.calculate(convertAngle(m_drive.getAngle()), m_drive.intakeSetpoint),
                true);
    }

    @Override
    public boolean isFinished(){
        return false;
    }
}