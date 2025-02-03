package frc.robot.systems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.AlgaeArmSubsystem;
import frc.robot.subsystems.CoralArmSubsystem;
import frc.robot.subsystems.CoralIntakeSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;

;

public class LoadingSystem
{

  private CoralArmSubsystem    m_coralArm;
  private AlgaeArmSubsystem    m_algaeArm;
  private ElevatorSubsystem    m_elevator;
  private CoralIntakeSubsystem m_wrist;


  public LoadingSystem(CoralArmSubsystem coralArm,
                       AlgaeArmSubsystem algaeArm,
                       ElevatorSubsystem elevator,
                       CoralIntakeSubsystem coralIntake)
  {
    m_coralArm = coralArm;
    m_algaeArm = algaeArm;
    m_elevator = elevator;
    m_wrist = coralIntake;
  }

  public Command coralLoad()
  {
    double coralArmLoadingAngleDegrees   = 45;
    double coralElevatorHighHeightMeters = 0.5;
    double coralElevatorLowHeightMeters  = 0;
    return (m_elevator.setElevatorHeight(coralElevatorHighHeightMeters)
                      .andThen(m_coralArm.setCoralArmAngle(coralArmLoadingAngleDegrees))
                      .andThen(Commands.waitUntil(m_coralArm::coralInLoadPosition))
                      .andThen(m_elevator.setElevatorHeight(coralElevatorLowHeightMeters))
                      .andThen(Commands.waitUntil(m_coralArm::coralLoaded))
                      .andThen(m_elevator.setElevatorHeight(coralElevatorHighHeightMeters))).alongWith(m_wrist.setWristAngle(90));
  }

  public Command algaeLoad()
  {
    // Put algae arm out, roll in
    double algaeArmLoadingAngleDegrees   = 45;
    double algaeElevatorHighHeightMeters = 2.0;
    double algaeElevatorLowHeightMeters  = 1.0;
    double straightWristAngle            = 90;

    return m_elevator.setElevatorHeight(algaeElevatorHighHeightMeters)
                     .andThen(m_algaeArm.setAlgaeArmAngle(algaeArmLoadingAngleDegrees))
                     .andThen(Commands.waitUntil(m_algaeArm::algaeInLoadPosition))
                     .andThen(m_elevator.setElevatorHeight(algaeElevatorLowHeightMeters))
                     .andThen(Commands.waitUntil(m_algaeArm::algaeLoaded))
                     .andThen(m_elevator.setElevatorHeight(algaeElevatorHighHeightMeters))
        ;
  }

  public Command coralLock()
  {
    // Set arm to target angle, elev target height
    double coralArmLockingAngleDegrees      = 45;
    double coralElevatorLockingHeightMeters = 3;
    return m_coralArm.setCoralArmAngle(coralArmLockingAngleDegrees)
                     .alongWith(m_elevator.setElevatorHeight(coralElevatorLockingHeightMeters));
  }

  public Command algaeLockProcessor()
  {
    // Set arm to target angle, elev target height
    double algaeArmLockingProcessorAngleDegrees      = 240;
    double algaeElevatorLockingProcessorHeightMeters = 2;

    return m_algaeArm.setAlgaeArmAngle(algaeArmLockingProcessorAngleDegrees)
                     .alongWith(m_elevator.setElevatorHeight(algaeElevatorLockingProcessorHeightMeters),
                                Commands.waitUntil(() -> {
                                  return
                                      m_algaeArm.aroundAngle(algaeElevatorLockingProcessorHeightMeters) &&
                                      m_elevator.aroundHeight(algaeElevatorLockingProcessorHeightMeters);
                                }));

  }

  public Command algaeLockNet()
  {
    // Set arm to target angle, elev target height
    double algaeArmLockingNetAngleDegrees      = 45;
    double algaeElevatorLockingNetHeightMeters = 0.5;
    return m_algaeArm.setAlgaeArmAngle(algaeArmLockingNetAngleDegrees)
                     .alongWith(m_elevator.setElevatorHeight(algaeElevatorLockingNetHeightMeters),
                                Commands.waitUntil(() -> {
                                  return
                                      m_algaeArm.aroundAngle(algaeArmLockingNetAngleDegrees) &&
                                      m_elevator.aroundHeight(algaeElevatorLockingNetHeightMeters);
                                }));
  }
}
