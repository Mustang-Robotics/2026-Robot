package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.DriveSubsystem;

public class HubDrive extends Command{
    double a;
    double b;
    double x = 4.626;
    double y = 4.035;
    DriveSubsystem m_drive;
    CommandXboxController m_controller;
    PIDController m_PID;

    public HubDrive(DriveSubsystem drive, CommandXboxController controller, PIDController PID){
        m_drive = drive;
        m_controller = controller;
        m_PID = PID;

        addRequirements(m_drive);
    }

    @Override
    public void initialize() {
        // Restore the odometry to the previously-saved pose when starting this mode
        m_drive.restorePose();
    }

    private double convertGyroAngle(double angle) {
        double result = angle % 360;
        if (result < 0) {
            result += 360;
        }
        return result;
    }

        private double findAngle(double x, double y) {
            double a = (x -= m_drive.getPose().getMeasureY().baseUnitMagnitude());
            double b = (y -= m_drive.getPose().getMeasureX().baseUnitMagnitude());
            return Math.atan2(a, b);
        }


    @Override
    public void execute(){
        m_drive.intakeSetpoint = convertGyroAngle(Math.toDegrees(findAngle(4.626, 4.035)));
        m_drive.drive(
                -MathUtil.applyDeadband(m_controller.getRawAxis(1), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_controller.getRawAxis(0), OIConstants.kDriveDeadband),
                m_PID.calculate(convertGyroAngle(m_drive.getAngle()), m_drive.intakeSetpoint),
                true);
    }

    @Override
    public boolean isFinished(){
        return false;
    }
}
