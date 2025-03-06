package frc.robot.systems;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.*;

;

public class LoadingSystem
{

  private CoralArmSubsystem    m_coralArm;
  private AlgaeArmSubsystem    m_algaeArm;
  private ElevatorSubsystem    m_elevator;
  private CoralIntakeSubsystem m_wrist;
  private TargetingSystem      m_targetSystem;
  private AlgaeIntakeSubsystem m_algaeIntake;
  private SwerveSubsystem m_swerve;


  public LoadingSystem(CoralArmSubsystem coralArm,
                       AlgaeArmSubsystem algaeArm,
                       ElevatorSubsystem elevator,
                       CoralIntakeSubsystem coralIntake,
                       TargetingSystem   targetSys,
                       AlgaeIntakeSubsystem algaeIntake,
                       SwerveSubsystem swerve)
  {
    m_coralArm = coralArm;
    m_algaeArm = algaeArm;
    m_elevator = elevator;
    m_wrist = coralIntake;
    m_targetSystem = targetSys;
    m_algaeIntake = algaeIntake;
    m_swerve = swerve;
  }

  //For testing, set the sensor to low voltage first
  //The elevator needs to rise first for the arm to come out

  public Command coralLoad()
  {
    double coralArmLoadingAngleDegrees   = -8;
    double coralStationHeightMeters = Units.feetToMeters(3) + Units.inchesToMeters(1.5);
    double coralElevatorHighHeightMeters = 0;

    return m_elevator.setElevatorHeight(coralElevatorHighHeightMeters)
                    .andThen(m_elevator.setElevatorHeight(coralElevatorHighHeightMeters).repeatedly()
                            .alongWith(m_coralArm.setCoralArmAngle(coralArmLoadingAngleDegrees).repeatedly())
                            .alongWith(m_wrist.setWristAngle(90)))
                     .until(() -> m_coralArm.coralLoaded());
  }

  public Command algaeLoad()//fix angle
  {

    return m_targetSystem.driveToCoralTarget(m_swerve)
                         .andThen(Commands.parallel(m_elevator.getCoralCommand(m_targetSystem).repeatedly(),
                                                    m_coralArm.getCoralCommand(m_targetSystem).repeatedly(),
                                                    m_swerve.lockPos())
                                          .until(m_elevator.atAlgaeHeight(m_targetSystem)
                                                           .and(m_algaeArm.atAlgaeAngle(m_targetSystem)))
                                          .withTimeout(5))
                                          .andThen(Commands.parallel(m_algaeIntake.setAlgaeIntakeRoller(IntakeConstants.AlgaeOuttakeSpeeds),
                                                                     m_elevator.getAlgaeCommand(m_targetSystem).repeatedly(),
                                                                     m_swerve.lockPos())
                                                           .withDeadline(m_algaeArm.load())
                                                           .withTimeout(1)
                                                           .until(() -> m_algaeArm.algaeLoaded())) .andThen(m_swerve.driveForwards()
                                                           .alongWith(m_elevator.getAlgaeCommand(m_targetSystem)
                                                                                .repeatedly())
                                                           .withTimeout(1)
                                                  );






    // Put algae arm out, roll in
    // double algaeArmLoadingAngleDegrees   = angle;
    // double elevatorExtendedHeightMeters = elevatorHeight;
            //Units.inchesToMeters(elevatorHeight) > ElevatorConstants.kElevatorUnextendedHeight?
            //Units.inchesToMeters(elevatorHeight) : 0 ;
    // double algaeElevatorHighHeightMeters =  elevatorExtendedHeightMeters;// The ball is higher than Branches
    // double algaeElevatorLowHeightMeters  = elevatorExtendedHeightMeters - Units.inchesToMeters(3.0);// change it back to 1.0

    // return m_elevator.setElevatorHeight(algaeElevatorHighHeightMeters)
    //                  .andThen(m_elevator.setElevatorHeight(algaeElevatorHighHeightMeters).repeatedly()
    //                          .alongWith(m_algaeArm.setAlgaeArmAngle(algaeArmLoadingAngleDegrees).repeatedly()))
    //                  .andThen(m_elevator.setElevatorHeight(algaeElevatorLowHeightMeters).repeatedly()
    //                           .alongWith(m_algaeArm.setAlgaeArmAngle(algaeArmLoadingAngleDegrees).repeatedly())
    //                          .alongWith(m_algaeIntake.setAlgaeIntakeRoller(0.5)))  // Remember to change the default intake speed
    //                  .until(() -> m_algaeArm.algaeLoaded())
    //                          .andThen(m_elevator.setElevatorHeight(algaeElevatorHighHeightMeters)
    //                                  .deadlineFor(m_algaeArm.setAlgaeArmAngle(algaeArmLoadingAngleDegrees).repeatedly()));

  }

  public Command coralLock()
  {
    // Set arm to target angle, elev target height
    return m_coralArm.getCoralCommand(m_targetSystem).repeatedly()
        .alongWith(m_elevator.getCoralCommand(m_targetSystem).repeatedly(), m_wrist.wristScore());
//    double coralArmLockingAngleDegrees      = m_targetSystem.getTargetBranchCoralArmAngle();
//    double coralElevatorLockingHeightMeters = m_targetSystem.getTargetBranchHeightMeters();
//    return m_elevator.setElevatorHeight(coralElevatorLockingHeightMeters)
//            .andThen(m_elevator.setElevatorHeight(coralElevatorLockingHeightMeters).repeatedly()
//                    .alongWith(m_coralArm.setCoralArmAngle(coralArmLockingAngleDegrees).repeatedly())
//                     .alongWith(m_wrist.setWristAngle(90).repeatedly()));

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
