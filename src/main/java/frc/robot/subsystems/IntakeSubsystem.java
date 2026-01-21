package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;

import frc.robot.Configs;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class IntakeSubsystem extends SubsystemBase {
    private final SparkMax m_intake = new SparkMax(7, MotorType.kBrushless);
    public RelativeEncoder shooterEncoder = m_intake.getEncoder();
    private SparkClosedLoopController shooterClosedLoopController = m_intake.getClosedLoopController();
    private double targetSpeed = 0;
    public IntakeSubsystem(){
        m_intake.configure(
          Configs.Intake.IntakeConfig,
          ResetMode.kResetSafeParameters,
          PersistMode.kPersistParameters);
    }
    public void setSpeed(double speed){
      targetSpeed = speed;

    }

    private void moveToSetpoint() {
    
    shooterClosedLoopController.setSetpoint(targetSpeed, ControlType.kMAXMotionVelocityControl);
  }

  @Override
  public void periodic() {
    moveToSetpoint();
  }
}