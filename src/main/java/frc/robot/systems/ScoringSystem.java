package frc.robot.systems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.subsystems.*;

public class ScoringSystem {
    private CoralArmSubsystem m_coralArm;
    private AlgaeIntakeSubsystem m_algaeIntake;
    private ElevatorSubsystem m_elevator;
    private SwerveSubsystem m_swerve;
    private AlgaeArmSubsystem m_algaeArm;

    public ScoringSystem(
            CoralArmSubsystem coralArm,
            ElevatorSubsystem elevator,
            SwerveSubsystem swerve,
            AlgaeIntakeSubsystem algaeIntake,
            AlgaeArmSubsystem algaeArm) {
        m_coralArm = coralArm;
        m_elevator = elevator;
        m_swerve = swerve;
        m_algaeIntake = algaeIntake;
        m_algaeArm = algaeArm;

    }
    public Command scoreCoral(){
        // Arm down, elevator down, drive backwards x in
        double coralArmAngleDegrees = 150;
        double elevatorHeightMeters = -20;
        double swervingDistance = 200;
        return m_coralArm.setCoralArmAngle(coralArmAngleDegrees)
                .alongWith(m_elevator.setElevatorHeight(elevatorHeightMeters))
                .andThen(m_swerve.scoreBackward().until(()->!m_coralArm.coralLoaded()));
    }

    public Command scoreAlgaeProcessor(){
//set elevator height, set alage height, spit out ball, drive pose?
        double algaeArmAngleDegrees = 240;
        double elevatorHeightMeters = 10;

        return m_algaeArm.setAlgaeArmAngle(algaeArmAngleDegrees)
                        .andThen(m_elevator.setElevatorHeight(elevatorHeightMeters))
                        .andThen(Commands.waitUntil(m_algaeArm::algaeInProcessorPosition))
                        .andThen(m_algaeIntake.setAlgaeIntakeRoller(Constants.IntakeConstants.AlgaeOutakeSpeeds));
    }
    public Command scoreAlgaeNet(){
        double algaeArmAngleDegrees = 90;
        double elevatorHeightMeters  = 30;
        return m_algaeArm.setAlgaeArmAngle(algaeArmAngleDegrees)
                .andThen(m_elevator.setElevatorHeight(elevatorHeightMeters))
                .andThen(Commands.waitUntil(m_algaeArm::algaeInNetPosition))
                .andThen(m_algaeIntake.setAlgaeIntakeRoller(Constants.IntakeConstants.AlgaeOutakeSpeeds));
                //testing-a different outake Speed for algae the net?
    }

}
