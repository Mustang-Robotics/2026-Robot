package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LauncherSubsystem;


public class ZeroShooter extends Command {
    private final LauncherSubsystem m_launcher;
    private final IntakeSubsystem m_intake;

    public ZeroShooter(LauncherSubsystem launcher, IntakeSubsystem intake) {
        m_launcher = launcher;
        m_intake = intake;
        addRequirements(m_launcher, m_intake);
    }

    @Override
    public void execute() {
        m_intake.changeSetpoint(0);
        m_launcher.feedOff();
        m_launcher.setSpeed(600);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
