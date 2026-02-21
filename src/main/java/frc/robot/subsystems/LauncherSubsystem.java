package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.spark.SparkMax;

import frc.robot.Configs;

public class LauncherSubsystem extends SubsystemBase{
    private final SparkMax m_launcher = new SparkMax(8, MotorType.kBrushless);
    private final SparkMax m_launcher_follower = new SparkMax(9, MotorType.kBrushless);
    private final SparkMax m_launcher_follower2 = new SparkMax(6, MotorType.kBrushless);
    private final SparkMax m_feeder = new SparkMax(33, MotorType.kBrushless);
    public double targetSpeed = 0;
    public RelativeEncoder shooterEncoder = m_launcher.getEncoder();
    private SparkClosedLoopController launcherClosedLoopController = m_launcher.getClosedLoopController();
    private final InterpolatingDoubleTreeMap rpmTable = new InterpolatingDoubleTreeMap();
    
    public LauncherSubsystem(){
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

        rpmTable.put(2.436, 2550.0);
        rpmTable.put(2.726, 2550.0);
        rpmTable.put(3.006, 2850.0);
        rpmTable.put(3.306, 2900.0);
        rpmTable.put(3.676, 3000.0);
        rpmTable.put(4.056, 3100.0);
        rpmTable.put(2.516, 2550.0);
        rpmTable.put(2.876, 2750.0);
        rpmTable.put(3.456, 2950.0);
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

    public double getRPMForDistance(double distance){
        return rpmTable.get(distance);
    }

    private void publishRPMSetpoint(){
        SmartDashboard.putNumber("Launcher RPM", shooterEncoder.getVelocity());
    }


    @Override
    public void periodic() {
        moveToSetpoint();
        publishRPMSetpoint();
    }

}

