package frc.robot.systems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Setpoints;
import frc.robot.subsystems.AlgaeArmSubsystem;
import frc.robot.subsystems.AlgaeIntakeSubsystem;
import frc.robot.subsystems.CoralArmSubsystem;
import frc.robot.subsystems.CoralIntakeSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

;

public class LoadingSystem
{

  private CoralArmSubsystem    m_coralArm;
  private AlgaeArmSubsystem    m_algaeArm;
  private ElevatorSubsystem    m_elevator;
  private CoralIntakeSubsystem m_wrist;
  private TargetingSystem      m_targetSystem;
  private AlgaeIntakeSubsystem m_algaeIntake;
  private CoralIntakeSubsystem m_coralIntake;
  private SwerveSubsystem      m_swerve;


  public LoadingSystem(CoralArmSubsystem coralArm,
                       AlgaeArmSubsystem algaeArm,
                       ElevatorSubsystem elevator,
                       CoralIntakeSubsystem coralIntake,
                       TargetingSystem targetSys,
                       AlgaeIntakeSubsystem algaeIntake,
                       SwerveSubsystem swerve)
  {
    m_coralArm = coralArm;
    m_algaeArm = algaeArm;
    m_elevator = elevator;
    m_wrist = coralIntake;
    m_targetSystem = targetSys;
    m_algaeIntake = algaeIntake;
    m_coralIntake = coralIntake;
    m_swerve = swerve;
  }

  //For testing, set the sensor to low voltage first
  //The elevator needs to rise first for the arm to come out


  public Command coralLoad()
  {

    return (Commands.parallel(m_elevator.CoralHP().repeatedly(), //Drive to HP and Move ELEVATOR AND ARM
                              m_coralArm.setCoralArmAngle(Setpoints.Arm.Coral.HP),
                              m_swerve.lockPos()))
        .until(m_elevator.aroundCoralHP()
                         .and(m_coralArm.aroundCoralHPAngle()))
        .withTimeout(5) //Move Intake angle to 0
        .andThen(m_coralIntake.wristRest().until(m_coralIntake.atRestingAngle()))
        .andThen(m_swerve.lockPos()
                         .withDeadline(m_coralArm.load()) //end command
                         .withTimeout(1)
                         .until(() -> m_coralArm.coralLoaded()));

  }

  public Command coralLoadAuto()
  {

    return (Commands.parallel(m_elevator.CoralHP().repeatedly(),
                              m_coralArm.setCoralArmAngle(Setpoints.Arm.Coral.HP).repeatedly()))
        .until(m_elevator.aroundCoralHP()
                         .and(m_coralArm.aroundCoralHPAngle()))
        .withTimeout(5) //Move Intake angle to 0
        .andThen(m_coralIntake.wristRest().until(m_coralIntake.atRestingAngle()))
        .withTimeout(1)
        .until(() -> m_coralArm.coralLoaded());

  }


  public Command algaeLoadAuto()
  {

    return Commands.parallel(m_elevator.getAlgaeCommand(m_targetSystem).repeatedly(),
                             m_algaeArm.getAlgaeCommand(m_targetSystem).repeatedly(),
                             m_swerve.lockPos())
                   .until(m_elevator.atAlgaeHeight(m_targetSystem)
                                    .and(m_algaeArm.atAlgaeAngle(m_targetSystem)))
                   .withTimeout(5)
                   .andThen(Commands.parallel(m_algaeIntake.setAlgaeIntakeRoller(IntakeConstants.AlgaeOuttakeSpeeds),
                                              m_elevator.getAlgaeCommand(m_targetSystem).repeatedly(),
                                              m_swerve.lockPos())
                                    .withDeadline(m_algaeArm.load())
                                    .withTimeout(1)
                                    .until(() -> m_algaeArm.algaeLoaded()))
                   .andThen((m_elevator.getAlgaeCommand(m_targetSystem).repeatedly())
                                .withTimeout(1));


  }

  public Command algaeLoad()
  {

    return m_targetSystem.driveToCoralTarget(m_swerve)
                         .andThen(Commands.parallel(m_elevator.getCoralCommand(m_targetSystem).repeatedly(),
                                                    m_algaeArm.getAlgaeCommand(m_targetSystem).repeatedly()
                                                   )
                                          .until(m_elevator.atAlgaeHeight(m_targetSystem)
                                                           .and(m_algaeArm.atAlgaeAngle(m_targetSystem)))
                                          .withTimeout(5))
                         .andThen(Commands.parallel(m_algaeIntake.setAlgaeIntakeRoller(IntakeConstants.AlgaeOuttakeSpeeds),
                                                    m_elevator.getAlgaeCommand(m_targetSystem).repeatedly())
                                          .withDeadline(m_algaeArm.load())
                                          .withTimeout(1)
                                          .until(() -> m_algaeArm.algaeLoaded()))
                         .andThen(m_swerve.driveForwards()
                                          .alongWith(m_elevator.getAlgaeCommand(m_targetSystem).repeatedly())
                                          .withTimeout(1));


  }


  public Command coralLock()
  {
    // Set arm to target angle, elev target height
    return m_coralArm.getCoralCommand(m_targetSystem).repeatedly()
                     .alongWith(m_elevator.getCoralCommand(m_targetSystem).repeatedly(), m_wrist.wristScore());
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
    double algaeElevatorLockingNetHeightMeters = Constants.ElevatorConstants.kMaxElevatorHeightMeters;
    return m_elevator.setElevatorHeight(algaeElevatorLockingNetHeightMeters)
                     .andThen(m_algaeArm.setAlgaeArmAngle(algaeArmLockingNetAngleDegrees).repeatedly());
  }
}
