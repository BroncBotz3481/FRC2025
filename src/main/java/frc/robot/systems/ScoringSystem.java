package frc.robot.systems;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.AlgaeArmSubsystem;
import frc.robot.subsystems.AlgaeIntakeSubsystem;
import frc.robot.subsystems.CoralArmSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.SwerveSubsystem;


public class ScoringSystem
{

  private CoralArmSubsystem    m_coralArm;
  private AlgaeIntakeSubsystem m_algaeIntake;
  private ElevatorSubsystem    m_elevator;
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
    // Arm down, elevator down, drive backwards x in?
    double coralArmAngleDegrees = m_targetSystem.getTargetBranchCoralArmAngle();
    double elevatorHeightMeters = m_targetSystem.getTargetBranchHeightMeters();

    return m_coralArm.setCoralArmAngle(coralArmAngleDegrees)
                     .alongWith(m_elevator.setElevatorHeight(elevatorHeightMeters))
                     .andThen(m_elevator.setElevatorHeight(
                         elevatorHeightMeters - Constants.ElevatorConstants.kLowerToScoreHeight))
                     .andThen(m_swerve.lockPos());
  }

  public Command scoreAlgaeProcessor()
  {
//set elevator height, set alage angle, spit out ball, drive pose?
    double swervingDistance = 10;
    return m_loadingSystem.algaeLockProcessor()//use targetting sys?
                          .andThen(m_algaeIntake.setAlgaeIntakeRoller(Constants.IntakeConstants.AlgaeOuttakeSpeeds)
                                                .until(() -> !m_algaeArm.algaeLoaded()));
  }

  public Command scoreAlgaeNet()
  {
    //set elevator height, set alage angle, spit out ball, drive pose?

    return m_loadingSystem.algaeLockNet()
                          .andThen(m_algaeIntake.setAlgaeIntakeRoller(Constants.IntakeConstants.AlgaeOuttakeSpeeds));
    //testing-a different outake Speed for algae the net?
  }

}
