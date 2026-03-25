package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;

public class LauncherSubsystem extends SubsystemBase {
    private final SparkMax m_launcher = new SparkMax(8, MotorType.kBrushless);
    private final SparkMax m_launcher_follower = new SparkMax(9, MotorType.kBrushless);
    private final SparkMax m_launcher_follower2 = new SparkMax(6, MotorType.kBrushless);
    private final SparkMax m_feeder = new SparkMax(33, MotorType.kBrushless);
    
    public double targetSpeed = 1000;
    public RelativeEncoder shooterEncoder = m_launcher.getEncoder();
    private SparkClosedLoopController launcherClosedLoopController = m_launcher.getClosedLoopController();

    // --- Jam Detection & Control Logic ---
    private final RelativeEncoder m_feederEncoder = m_feeder.getEncoder();
    private final Debouncer m_stallDebouncer = new Debouncer(0.2, DebounceType.kRising); 
    private final Timer m_reverseTimer = new Timer();
    
    private boolean m_feederRequested = false;
    private boolean m_manualReverseRequested = false; // New Manual Flag
    private boolean m_isAutoReversing = false;

    private final double STALL_THRESHOLD_RPM = 50.0;
    private final double AUTO_REVERSE_DURATION = 0.25; 
    private final double REVERSE_SPEED = -1.0;

    public LauncherSubsystem() {
        m_launcher.configure(
        Configs.Launcher.LauncherConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

        m_launcher_follower.configure(
        Configs.Launcher.Launcher_2Config,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

        m_launcher_follower2.configure(
        Configs.Launcher.Launcher_3Config,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

        m_feeder.configure(
        Configs.Launcher.FeederConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
    }

    public void setSpeed(double speed) {
        targetSpeed = speed;
    }

    public void feed() {
        m_feederRequested = true;
        m_manualReverseRequested = false;
    }

    public void feedReverse() {
        m_manualReverseRequested = true;
        m_feederRequested = false;
        m_isAutoReversing = false; // Cancel auto-logic if manual takes over
    }

    public void feedOff() {
        m_feederRequested = false;
        m_manualReverseRequested = false;
        m_isAutoReversing = false;
    }

    public boolean isJammed() {
        return m_isAutoReversing || m_manualReverseRequested;
    }

    private void handleFeederLogic() {
        // 1. Priority 1: Manual Override
        if (m_manualReverseRequested) {
            m_feeder.set(REVERSE_SPEED);
            return; // Exit early, manual wins
        }

        // 2. Priority 2: Auto-Clear Jam Logic
        boolean isStalled = m_feederRequested && (Math.abs(m_feederEncoder.getVelocity()) < STALL_THRESHOLD_RPM);

        if (m_stallDebouncer.calculate(isStalled) && !m_isAutoReversing) {
            m_isAutoReversing = true;
            m_reverseTimer.restart();
        }

        if (m_isAutoReversing) {
            m_feeder.set(REVERSE_SPEED);
            if (m_reverseTimer.hasElapsed(AUTO_REVERSE_DURATION)) {
                m_isAutoReversing = false;
            }
        } 
        // 3. Priority 3: Normal Operation
        else if (m_feederRequested) {
            m_feeder.set(1.0);
        } else {
            m_feeder.set(0);
        }
    }

    private void moveToSetpoint() {
        launcherClosedLoopController.setSetpoint(targetSpeed, ControlType.kMAXMotionVelocityControl);
    }

    @Override
    public void periodic() {
        moveToSetpoint();
        handleFeederLogic();
    }
}