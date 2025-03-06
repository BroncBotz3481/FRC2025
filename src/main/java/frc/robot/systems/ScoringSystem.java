package frc.robot.systems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
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

  ///  Score Coral Command for PathPlanner that does not move the swerve drive at all keeping the pathplanner auto
  /// intact.
  public Command scoreCoralAuto()
  {
    return Commands.parallel(m_elevator.getCoralCommand(m_targetSystem).repeatedly(),
                             m_coralArm.getCoralCommand(m_targetSystem).repeatedly())
                   .until(m_elevator.atCoralHeight(m_targetSystem)
                                    .and(m_coralArm.atCoralAngle(m_targetSystem)))
                   .withTimeout(5)
                   .andThen(m_coralIntake.wristScore().until(m_coralIntake.atScoringAngle()))
                   .andThen(Commands.parallel(m_coralIntake.wristScore(),
                                              m_elevator.getCoralCommand(m_targetSystem).repeatedly())
                                    .withDeadline(m_coralArm.score())
                                    .withTimeout(1)
                                    .until(() -> m_coralArm.coralScored()));
  }

  public Command scoreCoral()
  {
    // Arm down, elevator down, drive backwards x in
    return m_targetSystem.driveToCoralTarget(m_swerve)
                         .andThen(Commands.parallel(m_elevator.getCoralCommand(m_targetSystem).repeatedly(),
                                                    m_coralArm.getCoralCommand(m_targetSystem).repeatedly(),
                                                    m_swerve.lockPos())
                                          .until(m_elevator.atCoralHeight(m_targetSystem)
                                                           .and(m_coralArm.atCoralAngle(m_targetSystem)))
                                          .withTimeout(5))
                         .andThen(m_coralIntake.wristScore().until(m_coralIntake.atScoringAngle()))
                         .andThen(Commands.parallel(m_coralIntake.wristScore(),
                                                    m_elevator.getCoralCommand(m_targetSystem).repeatedly(),
                                                    m_swerve.lockPos())
                                          .withDeadline(m_coralArm.score())
                                          .withTimeout(1)
                                          .until(() -> m_coralArm.coralScored()))
                         .andThen(m_swerve.driveForwards()
                                          .alongWith(m_elevator.getCoralCommand(m_targetSystem)
                                                               .repeatedly(),
                                                     m_coralIntake.wristScore())
                                          .withTimeout(1)
                                 );
  }

  ///  Autonomous command for scoring the algae arm
  public Command scoreAlgaeProcessorAuto()
  {
    //set elevator height, set algae angle, spit out ball, drive pose
    return Commands.parallel(m_elevator.AlgaePROCESSOR().repeatedly(),
                             m_algaeArm.PROCESSOR().repeatedly())
                   .until(m_elevator.aroundAlgaePROCESSOR().and(m_algaeArm.aroundPROCESSORAngle()))
                   .withTimeout(5)
                   .andThen(Commands.parallel(m_algaeIntake.out(),
                                              m_elevator.AlgaePROCESSOR().repeatedly(),
                                              m_algaeArm.PROCESSOR().repeatedly())
                                    .until(() -> m_algaeArm.algaeScored())
                                    .withTimeout(1));
  }

  public Command scoreAlgaeProcessor()
  {
    //set elevator height, set algae angle, spit out ball, drive pose
    return m_swerve.driveToProcessor()
                   .andThen(Commands.parallel(m_elevator.AlgaePROCESSOR().repeatedly(),
                                              m_algaeArm.PROCESSOR().repeatedly(),
                                              m_swerve.lockPos())
                                    .until(m_elevator.aroundAlgaePROCESSOR().and(m_algaeArm.aroundPROCESSORAngle()))
                                    .withTimeout(5))
                   .andThen(Commands.parallel(m_algaeIntake.out(),
                                              m_elevator.AlgaePROCESSOR().repeatedly(),
                                              m_algaeArm.PROCESSOR().repeatedly(),
                                              m_swerve.lockPos())
                                    .until(() -> m_algaeArm.algaeScored())
                                    .withTimeout(1)
                           );

  }

  public Command scoreAlgaeNet()
  {
    //set elevator height, set alage angle, spit out ball, drive pose
    return Commands.parallel(m_algaeArm.NET().repeatedly(),
                             m_elevator.AlgaeNET().repeatedly())
                   .withTimeout(5)
                   .until(m_elevator.aroundAlgaeNET().and(m_algaeArm.aroundNETAngle()))
                   .andThen(Commands.parallel(m_algaeArm.NET().repeatedly(),
                                              m_elevator.AlgaeNET().repeatedly(),
                                              m_algaeIntake.out())
                                    .withTimeout(2)
                                    .until(() -> m_algaeArm.algaeScored()));
  }

}






//                     WALDO
///                     ( )
///                   /-----\
///                  |||||||||
//                   ((o)-(o))
//                    |  W  |
//                    --| |--