package frc.robot.systems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.subsystems.AlgaeArmSubsystem;
import frc.robot.subsystems.AlgaeIntakeSubsystem;
import frc.robot.subsystems.CoralArmSubsystem;
import frc.robot.subsystems.CoralIntakeSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import swervelib.SwerveInputStream;


public class ScoringSystem
{

  private CoralArmSubsystem    m_coralArm;
  private AlgaeIntakeSubsystem m_algaeIntake;
  private ElevatorSubsystem    m_elevator;
  private SwerveSubsystem      m_swerve;
  private SwerveInputStream    m_swerveInputStream;
  private LoadingSystem        m_loadingSystem;
  private AlgaeArmSubsystem    m_algaeArm;
  private TargetingSystem      m_targetSystem;
  private CoralIntakeSubsystem m_coralIntake;

  public ScoringSystem(
      CoralArmSubsystem coralArm,
      ElevatorSubsystem elevator,
      SwerveSubsystem swerve,
      AlgaeIntakeSubsystem algaeIntake,
      AlgaeArmSubsystem algaeArm,
      LoadingSystem loading, TargetingSystem targeting,
      CoralIntakeSubsystem coralIntake,
      SwerveInputStream driveStream)
  {
    m_coralArm = coralArm;
    m_elevator = elevator;
    m_swerve = swerve;
    m_algaeIntake = algaeIntake;
    m_loadingSystem = loading;
    m_algaeArm = algaeArm;
    m_targetSystem = targeting;
    m_coralIntake = coralIntake;
    m_swerveInputStream = driveStream;
  }

  public Command scoreCoral()
  {
    // Arm down, elevator down, drive backwards x in
    return m_targetSystem.driveToTarget(m_swerve)
                         .andThen(
                             Commands.parallel(m_elevator.getCoralCommand(m_targetSystem).repeatedly(),
                                               m_coralArm.getCoralCommand(m_targetSystem).repeatedly(),
                                               m_coralIntake.wristScore()).withTimeout(2)
                                     .until(m_elevator.atCoralHeight(m_targetSystem)
                                                      .and(m_coralArm.atCoralAngle(m_targetSystem)))
                                     .andThen(m_coralArm.score())
                                     .alongWith(m_swerve.lockPos())
                                     .until(() -> m_coralArm.coralScored()));

//    return new ParallelDeadlineGroup(
//        m_elevator.setElevatorHeight(elevatorHeightMeters).withName("ScoreCoralElevatorHeight")
//                  .andThen(m_coralIntake.spitCoralOut(IntakeConstants.defaultrRollerSpeed, 90))
//                  .andThen(Commands.print("Tell me why aint nothing but a mistake"))
//                  .andThen(m_elevator.setElevatorHeight(
//                                         elevatorHeightMeters - Constants.ElevatorConstants.kLowerToScoreHeight)
//                                     .withName("ScoreCoralElevatorHeightLower"))
//                  //.alongWith(m_coralArm.setCoralArmAngle(coralArmAngleDegrees)).repeatedly()
//                  .andThen(Commands.print("Tell me why aint nothing but an heart ache")),
//        m_coralArm.setCoralArmAngle(coralArmAngleDegrees).withName("ScoreCoralArmAngle").repeatedly(),
//        m_swerve.lockPos().withName("LockPose")
//    );
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
    double algaeArmAngleDegrees = 48;
    double elevatorHeightMeters = 42;
    return m_algaeArm.setAlgaeArmAngle(algaeArmAngleDegrees).repeatedly()
                     .alongWith(m_elevator.setElevatorHeight(elevatorHeightMeters))
                     .until(() -> m_elevator.aroundHeight(elevatorHeightMeters))
                     .andThen(m_algaeIntake.setAlgaeIntakeRoller(Constants.IntakeConstants.AlgaeOuttakeSpeeds))
                     .until(() -> !m_algaeArm.algaeLoaded());
  }

}
