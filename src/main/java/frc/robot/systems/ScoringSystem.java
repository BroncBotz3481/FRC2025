package frc.robot.systems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import frc.robot.Constants;
import frc.robot.subsystems.*;


public class ScoringSystem
{

  private CoralArmSubsystem    m_coralArm;
  private AlgaeIntakeSubsystem m_algaeIntake;
  private ElevatorSubsystem m_elevator;
  private SwerveSubsystem      m_swerve;
  private LoadingSystem        m_loadingSystem;
  private AlgaeArmSubsystem    m_algaeArm;
  private TargetingSystem      m_targetSystem;

  public ScoringSystem(
      CoralArmSubsystem coralArm,
      ElevatorSubsystem elevator,
      SwerveSubsystem swerve,
      AlgaeIntakeSubsystem algaeIntake,
      AlgaeArmSubsystem algaeArm,
      LoadingSystem loading, TargetingSystem targeting)
  {
    m_coralArm = coralArm;
    m_elevator = elevator;
    m_swerve = swerve;
    m_algaeIntake = algaeIntake;
    m_loadingSystem = loading;
    m_algaeArm = algaeArm;
    m_targetSystem = targeting;

  }

  public Command scoreCoral()
  {
    // Arm down, elevator down, drive backwards x in
    double coralArmAngleDegrees = m_targetSystem.getTargetBranchCoralArmAngle();
    double elevatorHeightMeters = m_targetSystem.getTargetBranchHeightMeters();

    return new ParallelDeadlineGroup(
      m_elevator.setElevatorHeight(elevatorHeightMeters).withName("ScoreCoralElevatorHeight")
    .andThen(Commands.print("Tell me why aint nothing but a mistake"))
    .andThen(m_elevator.setElevatorHeight(
        elevatorHeightMeters - Constants.ElevatorConstants.kLowerToScoreHeight).withName("ScoreCoralElevatorHeightLower"))
     //.alongWith(m_coralArm.setCoralArmAngle(coralArmAngleDegrees)).repeatedly()
        .andThen(Commands.print("Tell me why aint nothing but an heart ache")),
      m_coralArm.setCoralArmAngle(coralArmAngleDegrees).withName("ScoreCoralArmAngle").repeatedly(),
                    m_swerve.lockPos().withName("LockPose")
                     );
  }

  public Command scoreAlgaeProcessor()
  {
    //set elevator height, set algae angle, spit out ball, drive pose
    double algaeArmAngleDegrees = -45;
    double elevatorHeightMeters = 1.0;
    return m_algaeArm.setAlgaeArmAngle(algaeArmAngleDegrees).repeatedly()
            .alongWith(m_elevator.setElevatorHeight(elevatorHeightMeters))
            .until(() -> m_elevator.aroundHeight(elevatorHeightMeters))
            .andThen(m_algaeIntake.setAlgaeIntakeRoller(Constants.IntakeConstants.AlgaeOuttakeSpeeds)
            .until(() -> !m_algaeArm.algaeLoaded()));
  }

  public Command scoreAlgaeNet()
  {
    //set elevator height, set alage angle, spit out ball, drive pose
    double algaeArmAngleDegrees = 45;
    double elevatorHeightMeters = 4.0;
    return m_algaeArm.setAlgaeArmAngle(algaeArmAngleDegrees).repeatedly()
            .alongWith(m_elevator.setElevatorHeight(elevatorHeightMeters))
            .until(() -> m_elevator.aroundHeight(elevatorHeightMeters))
            .andThen(m_algaeIntake.setAlgaeIntakeRoller(Constants.IntakeConstants.AlgaeOuttakeSpeeds))
            .until(() -> !m_algaeArm.algaeLoaded());
  }

}
