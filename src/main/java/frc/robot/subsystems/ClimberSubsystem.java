package frc.robot.subsystems;


import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ClimberConstants;
import frc.robot.HWMap.Climber;

public class ClimberSubsystem extends SubsystemBase
{

  // With eager singleton initialization, any static variables/fields used in the
  // constructor must appear before the "INSTANCE" variable so that they are initialized
  // before the constructor is called when the "INSTANCE" variable initializes.

  private final SparkMax m_ClimberMotor      = new SparkMax(Climber.climberLeftMotorID, MotorType.kBrushless);
  private final SparkClosedLoopController m_controller = m_ClimberMotor.getClosedLoopController();
  private final RelativeEncoder m_encoder    = m_ClimberMotor.getEncoder();
  private final SparkMax m_ClimberMotorRight = new SparkMax(Climber.climberRightMotorID, MotorType.kBrushless);
  public final Trigger isDown = new Trigger(()->getPosition()<=-0.2);
  public final Trigger isUp = new Trigger(()->getPosition()>0);

    public ClimberSubsystem()
    {
      SparkMaxConfig cfg = new SparkMaxConfig();
      cfg.smartCurrentLimit(40)
      .idleMode(IdleMode.kBrake)
      .closedLoop.pid(3, 0, 0);
      m_ClimberMotor.configure(cfg, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
      cfg.follow(m_ClimberMotor, true);
      m_ClimberMotorRight.configure(cfg, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    }
  
    public Command down()
    {
      return setPOwer(-0.5).until(isDown);
    }


  
  public Command up()
  {
    return setPOwer(0.5).until(isUp);

  }

  public Command climbUp()
  {
    return run(() -> {
      m_ClimberMotor.set(ClimberConstants.kClimbSpeed);
    });
  }

  public Command climbDown()
  {
    return run(() -> {
      m_ClimberMotor.set(-ClimberConstants.kClimbSpeed);
    });
  }

  public double getPosition()
  {
    return m_encoder.getPosition() / 81.0;
  }

  @Override
  public void periodic()
  {
    SmartDashboard.putNumber("Climber Rotation (Rotations)", getPosition());
  }

  public Command setPOwer(double p)
  {
    return run(()->m_ClimberMotor.set(p));
  }
  
}

