package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;

import frc.robot.Configs;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class IntakeSubsystem extends SubsystemBase {

    private final SparkMax m_extend = new SparkMax(34, MotorType.kBrushless);
    private SparkClosedLoopController ExtendClosedLoopController = m_extend.getClosedLoopController();
    public final AbsoluteEncoder extendEncoder = m_extend.getAbsoluteEncoder();
    public double extendTarget = .2;

    private final SparkMax m_intake = new SparkMax(7, MotorType.kBrushless);

    

    public IntakeSubsystem(){
        m_intake.configure(
          Configs.Intake.IntakeConfig,
          ResetMode.kResetSafeParameters,
          PersistMode.kPersistParameters);

        m_extend.configure(
          Configs.Intake.ExtendConfig,
          ResetMode.kResetSafeParameters,
          PersistMode.kPersistParameters);

    }

    public void setPercent(double percent){
      m_intake.set(percent);
    }

  public void changeSetpoint(double setpoint) {
    extendTarget = setpoint;
  }

  private void moveToSetpoint() {
    ExtendClosedLoopController.setSetpoint(
      extendTarget, ControlType.kMAXMotionPositionControl, ClosedLoopSlot.kSlot0, .22*calcArmAngle());
  }

  private double calcArmAngle() {
    return Math.cos(extendEncoder.getPosition()*2*Math.PI);
  }



  @Override
  public void periodic() {
    moveToSetpoint();
  }
}