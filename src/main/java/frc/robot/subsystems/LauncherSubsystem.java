package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.spark.SparkMax;

import frc.robot.Configs;

public class LauncherSubsystem extends SubsystemBase{
    private final SparkMax m_launcher = new SparkMax(8, MotorType.kBrushless);
    private final SparkMax m_launcher_follower = new SparkMax(9, MotorType.kBrushless);
    private final SparkMax m_feeder = new SparkMax(33, MotorType.kBrushless);
    public double targetSpeed = 0;
    public RelativeEncoder shooterEncoder = m_launcher.getEncoder();
    private SparkClosedLoopController launcherClosedLoopController = m_launcher.getClosedLoopController();
    public LauncherSubsystem(){
        m_launcher.configure(
          Configs.Launcher.LauncherConfig,
          ResetMode.kResetSafeParameters,
          PersistMode.kPersistParameters);
        
        m_launcher_follower.configure(
            Configs.Launcher.Launcher_2Config,
            ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters);
        }
    public void setSpeed(double speed){
        targetSpeed = speed;

    }

    public void feed(){
        m_feeder.set(1);

    }
    public void feedOff(){
        m_feeder.set(0);

    }

    private void moveToSetpoint() {
        launcherClosedLoopController.setSetpoint(targetSpeed, ControlType.kMAXMotionVelocityControl);
    }

    @Override
    public void periodic() {
        moveToSetpoint();
    }

}

