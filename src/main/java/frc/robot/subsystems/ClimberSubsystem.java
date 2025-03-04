package frc.robot.subsystems;


import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.IntakeConstants;

public class ClimberSubsystem extends SubsystemBase
{

  // With eager singleton initialization, any static variables/fields used in the
  // constructor must appear before the "INSTANCE" variable so that they are initialized
  // before the constructor is called when the "INSTANCE" variable initializes.

 private final SparkMax m_motor = new SparkMax(ClimberConstants.climberMotorID, MotorType.kBrushless);
 private final SparkMax m_motorRt = new SparkMax(ClimberConstants.climberMotorRightID, MotorType.kBrushless);//right motor


  public ClimberSubsystem()
  {
      SparkMaxConfig m_config = new SparkMaxConfig();
      m_config.smartCurrentLimit(Constants.ClimberConstants.kClimberCurrentLimit)
              .idleMode(IdleMode.kBrake);//set up closeLoop?
      m_motor.configure(m_config, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);

      SparkMaxConfig m_configRt = new SparkMaxConfig();
      m_configRt.smartCurrentLimit(Constants.ClimberConstants.kClimberCurrentLimit)
                .idleMode(IdleMode.kBrake)
                .follow(m_motor, true);
      m_motorRt.configure(m_configRt, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);

  }

  public Command climbUp()
  {
    return run(() -> {
      m_motor.set(ClimberConstants.kClimbSpeed);
      
    });
  }

  public Command climbDown()
  {
    return run(() -> {
      m_motor.set(-ClimberConstants.kClimbSpeed);
    });
  }
  
  public Command stop()
  {
    return run(() -> {
      m_motor.set(0);
    });
  }
}

