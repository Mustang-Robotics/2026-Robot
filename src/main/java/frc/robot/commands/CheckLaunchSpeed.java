package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LauncherSubsystem;


public class CheckLaunchSpeed extends Command {
    private final LauncherSubsystem m_launcher;

    public CheckLaunchSpeed(LauncherSubsystem launcher) {
        m_launcher = launcher;
        addRequirements(m_launcher);
    }

    @Override
    public boolean isFinished() {
        return MathUtil.isNear(m_launcher.targetSpeed,m_launcher.shooterEncoder.getVelocity(),1000);
    }
}