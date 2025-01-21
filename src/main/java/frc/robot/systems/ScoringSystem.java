package frc.robot.systems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.subsystems.*;

import javax.swing.*;


public class ScoringSystem {
    private CoralArmSubsystem m_coralArm;
    private AlgaeIntakeSubsystem m_algaeIntake;
    private ElevatorSubsystem m_elevator;
    private SwerveSubsystem m_swerve;
    private LoadingSystem m_loadingSystem;

    public ScoringSystem(
            CoralArmSubsystem coralArm,
            ElevatorSubsystem elevator,
            SwerveSubsystem swerve,
            AlgaeIntakeSubsystem algaeIntake,
            AlgaeArmSubsystem algaeArm,
            LoadingSystem loading) {
        m_coralArm = coralArm;
        m_elevator = elevator;
        m_swerve = swerve;
        m_algaeIntake = algaeIntake;
        m_loadingSystem = loading;

    }
    public Command scoreCoral(){
        // Arm down, elevator down, drive backwards x in?
        double coralArmAngleDegrees = 150;
        double elevatorHeightMeters = -20;

        return m_coralArm.setCoralArmAngle(coralArmAngleDegrees)
                .alongWith(m_elevator.setElevatorHeight(elevatorHeightMeters))
                .andThen(m_swerve.scoreBackward().until(()->!m_coralArm.coralLoaded()));
    }

    public Command scoreAlgaeProcessor(){
//set elevator height, set alage angle, spit out ball, drive pose?
        double swervingDistance = 10;
        return m_loadingSystem.algaeLockProcessor()
                .andThen(m_algaeIntake.setAlgaeIntakeRoller(Constants.IntakeConstants.AlgaeOuttakeSpeeds));
    }
    public Command scoreAlgaeNet(){
        //set elevator height, set alage angle, spit out ball, drive pose?

        return m_loadingSystem.algaeLockNet()
                .andThen(m_algaeIntake.setAlgaeIntakeRoller(Constants.IntakeConstants.AlgaeOuttakeSpeeds));
                //testing-a different outake Speed for algae the net?
    }

}
