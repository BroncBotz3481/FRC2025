package frc.robot.subsystems;


import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.IntakeConstants;

public class ClimberSubsystem extends SubsystemBase
{

  // With eager singleton initialization, any static variables/fields used in the
  // constructor must appear before the "INSTANCE" variable so that they are initialized
  // before the constructor is called when the "INSTANCE" variable initializes.

 private final SparkMax m_ClimberMotor = new SparkMax(ClimberConstants.climberMotorID, MotorType.kBrushless);
  public ClimberSubsystem()
  {
    // TODO: Set the default command, if any, for this subsystem by calling setDefaultCommand(command) Done
    //       in the constructor or in the robot coordination class, such as RobotContainer.
    //       Also, you can call addChild(name, sendableChild) to associate sendables with the subsystem
    //       such as SpeedControllers, Encoders, DigitalInputs, etc.
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
  
  public Command stop()
  {
    return run(() -> {
      m_ClimberMotor.set(0);
    });
  }
}

