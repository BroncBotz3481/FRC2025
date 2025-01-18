package frc.robot.systems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.CoralArmSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class ScoringSystem {
    private CoralArmSubsystem m_coralArm;
    private ElevatorSubsystem m_elevator;
    private SwerveSubsystem m_swerve;

    public ScoringSystem(CoralArmSubsystem coralArm, ElevatorSubsystem elevator, SwerveSubsystem swerve) {
        m_coralArm = coralArm;
        m_elevator = elevator;
        m_swerve = swerve;

    }
    public Command scoreCoral(){
        // Arm down, elevator down, drive backwards x in
        double coralArmAngleDegrees = 150;
        double elevatorHeight = -20;
        double swervingDistance = 200;
        return m_coralArm.setCoralArmAngle(coralArmAngleDegrees)
                .alongWith(m_elevator.setElevatorHeight(elevatorHeight))
                .andThen(m_swerve.scoreBackward().until(()->!m_coralArm.coralLoaded()));
    }

    public Command scoreAlgaeProcessor(){
        return Commands.none();
    }
    public Command scoreAlgaeNet(){
        return Commands.none();
    }

}
