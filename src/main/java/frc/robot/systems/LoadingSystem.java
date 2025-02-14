package frc.robot.systems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.subsystems.AlgaeArmSubsystem;
import frc.robot.subsystems.CoralArmSubsystem;
import frc.robot.subsystems.CoralIntakeSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.Constants.CoralArmConstants;

;

public class LoadingSystem
{

  private CoralArmSubsystem    m_coralArm;
  private AlgaeArmSubsystem    m_algaeArm;
  private ElevatorSubsystem    m_elevator;
  private CoralIntakeSubsystem m_wrist;
  private TargetingSystem      m_targetSystem;


  public LoadingSystem(CoralArmSubsystem coralArm,
                       AlgaeArmSubsystem algaeArm,
                       ElevatorSubsystem elevator,
                       CoralIntakeSubsystem coralIntake,
                       TargetingSystem   targetSys)
  {
    m_coralArm = coralArm;
    m_algaeArm = algaeArm;
    m_elevator = elevator;
    m_wrist = coralIntake;
    m_targetSystem = targetSys;
  }

  public Command coralLoad()
  {
    double coralArmLoadingAngleDegrees   = 45;
    double coralArmDefaultAngleDegrees   = -90;
    double coralElevatorHighHeightMeters = 2.0;
    double coralElevatorLowHeightMeters  = 1.0;

    //to maintain a certain height/angle
    return  m_elevator.setElevatorHeight(coralElevatorHighHeightMeters).repeatedly()
                      .alongWith(m_coralArm.setCoralArmAngle(coralArmLoadingAngleDegrees).repeatedly())
                      .until(() -> m_coralArm.coralInLoadPosition() && m_coralArm.coralLoaded());
  }

  public Command algaeLoad()
  {
    // Put algae arm out, roll in
    double algaeArmLoadingAngleDegrees   = 15;
    double algaeElevatorHighHeightMeters = m_targetSystem.getTargetBranchHeightMeters();
    double algaeElevatorLowHeightMeters  = algaeElevatorHighHeightMeters - 0.25;//0.25-elev height change
    double straightWristAngle            = 90;

    return m_elevator.setElevatorHeight(algaeElevatorHighHeightMeters)
                     .alongWith(m_algaeArm.setAlgaeArmAngle(algaeArmLoadingAngleDegrees).repeatedly())
                     .until(() ->m_elevator.aroundHeight(algaeElevatorHighHeightMeters))//interupt the setElev Cmd while setAngle still running
                     .andThen(Commands.waitUntil(m_algaeArm::algaeInLoadPosition))//.until(() ->m_algaeArm.algaeInLoadPosition()))
                     .andThen(m_elevator.setElevatorHeight(algaeElevatorLowHeightMeters))
                     .andThen(Commands.waitUntil(m_algaeArm::algaeLoaded))
                     .andThen(m_elevator.setElevatorHeight(algaeElevatorHighHeightMeters));

  }

  public Command coralLock()
  {
    // Set arm to target angle, elev target height
    double coralArmLockingAngleDegrees      = m_targetSystem.getTargetBranchCoralArmAngle();
    double coralElevatorLockingHeightMeters = m_targetSystem.getTargetBranchHeightMeters();
    return m_elevator.setElevatorHeight(coralElevatorLockingHeightMeters).repeatedly()
                     .alongWith(m_coralArm.setCoralArmAngle(coralArmLockingAngleDegrees).repeatedly())
                     .alongWith(m_wrist.setWristAngle(90).repeatedly());
  }

  public Command algaeLockProcessor()
  {
    // Set arm to target angle, elev target height
    double algaeArmLockingProcessorAngleDegrees      = -45;
    double algaeElevatorLockingProcessorHeightMeters = 1.0;

    return m_elevator.setElevatorHeight(algaeElevatorLockingProcessorHeightMeters)
                     .andThen(m_algaeArm.setAlgaeArmAngle(algaeArmLockingProcessorAngleDegrees).repeatedly());
  }


  public Command algaeLockNet()
  {
    // Set arm to target angle, elev target height
    double algaeArmLockingNetAngleDegrees      = 45;
    double algaeElevatorLockingNetHeightMeters = 4.0;
    return m_elevator.setElevatorHeight(algaeElevatorLockingNetHeightMeters)
                     .andThen(m_algaeArm.setAlgaeArmAngle(algaeArmLockingNetAngleDegrees).repeatedly());
  }
}
