package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.DriveSubsystem;

public class OrientDrive extends Command{
    DriveSubsystem m_drive;
    CommandXboxController m_controller;
    ProfiledPIDController m_PID;
    double m_angle;

    public OrientDrive(DriveSubsystem drive, CommandXboxController controller, ProfiledPIDController PID, double angle){
        m_drive = drive;
        m_controller = controller;
        m_PID = PID;
        m_angle = angle;

        addRequirements(m_drive);
    }

    private double convertAngle(double angle) {
        double result = angle % 360;
        if (result < 0) {
            result += 360;
        }
        return result;
    }


    private double allianceCheck() {
        boolean red = false;
        var alliance = DriverStation.getAlliance();
        double plusAngle = 0;
        if(alliance.isPresent()) {
          red = alliance.get() == DriverStation.Alliance.Red;
        }
        if(red){
          plusAngle = 180;
        }else {
            plusAngle = 0;
        }

        return plusAngle;
    }

    @Override
    public void execute(){
        // Deadband left-stick inputs once and reuse
        double xInput = MathUtil.applyDeadband(m_controller.getRawAxis(1), OIConstants.kDriveDeadband);
        double yInput = MathUtil.applyDeadband(m_controller.getRawAxis(0), OIConstants.kDriveDeadband);

        m_drive.rotationSetpoint = convertAngle(m_angle + allianceCheck());

        // Only apply PID rotation when there is a translation input on the left stick
        double rot = m_PID.calculate(convertAngle(m_drive.getAngle()), m_drive.rotationSetpoint);


        m_drive.drive(
                -xInput,
                -yInput,
                rot,
                true);
    }

    @Override
    public boolean isFinished(){
        return false;
    }
}