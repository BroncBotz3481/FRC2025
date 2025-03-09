package frc.robot.subsystems;


import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;
import frc.robot.HWMap.Climber;

public class ClimberSubsystem extends SubsystemBase
{

  // With eager singleton initialization, any static variables/fields used in the
  // constructor must appear before the "INSTANCE" variable so that they are initialized
  // before the constructor is called when the "INSTANCE" variable initializes.

  private final SparkMax m_ClimberMotor      = new SparkMax(Climber.climberLeftMotorID, MotorType.kBrushless);
  private final RelativeEncoder m_encoder    = m_ClimberMotor.getEncoder();
  private final SparkMax m_ClimberMotorRight = new SparkMax(Climber.climberRightMotorID, MotorType.kBrushless);
  private boolean isDown = false;
    public ClimberSubsystem()
    {
      SparkMaxConfig cfg = new SparkMaxConfig();
      cfg.smartCurrentLimit(40)
      .idleMode(IdleMode.kBrake);
      m_ClimberMotor.configure(cfg, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
      cfg.follow(m_ClimberMotor, true);
      m_ClimberMotorRight.configure(cfg, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    }
  
    public Command down()
    {
      return runEnd(()->m_ClimberMotor.set(-0.5),()->{isDown = true;}).onlyIf(()->!isDown).withTimeout(0.5);
  }

  public Command up()
  {
    return runEnd(()->m_ClimberMotor.set(0.5),()->{isDown=false;}).onlyIf(()->isDown).withTimeout(0.5);
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

  @Override
  public void periodic()
  {
    SmartDashboard.putNumber("Climber Rotation (Rotations)", m_encoder.getPosition() / 81);
  }

  public Command setPOwer(double p)
  {
    return run(()->m_ClimberMotor.set(p));
  }

  public Command stop()
  {
    return run(() -> {
      m_ClimberMotor.set(0);
    });
  }
}

