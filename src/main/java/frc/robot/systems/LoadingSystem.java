package frc.robot.systems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.AlgaeArmSubsystem;
import frc.robot.subsystems.CoralArmSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;

public class LoadingSystem {

    private CoralArmSubsystem m_coralArm;
    private AlgaeArmSubsystem m_algaeArm;
    private ElevatorSubsystem m_elevator;

    public LoadingSystem(CoralArmSubsystem coralArm, AlgaeArmSubsystem algaeArm, ElevatorSubsystem elevator) {
        m_coralArm = coralArm;
        m_algaeArm = algaeArm;
        m_elevator = elevator;
    }

    public Command coralLoad()
    {
        double coralArmLoadingAngleDegrees = 90;
        double coralElevatorHighHeightMeters = 3;
        double coralElevatorLowHeightMeters = 1;
        return m_elevator.setElevatorHeight(coralElevatorHighHeightMeters)
                .andThen(m_coralArm.setCoralArmAngle(coralArmLoadingAngleDegrees))
                .andThen(Commands.waitUntil(m_coralArm::coralInLoadPosition))
                .andThen(m_elevator.setElevatorHeight(coralElevatorLowHeightMeters))
                .andThen(Commands.waitUntil(m_coralArm::coralLoaded))
                .andThen(m_elevator.setElevatorHeight(coralElevatorHighHeightMeters));
    }

    public Command algaeLoad(){
        // Put algae arm out, roll in
        return Commands.none();
    }

    public Command coralLock(){
        // Set arm to target angle, elev target height
        return Commands.none();
    }

    public Command algaeLockProcessor(){
        // Set arm to target angle, elev target height
        return Commands.none();
    }

    public Command algaeLockNet(){
        // Set arm to target angle, elev target height
        return Commands.none();
    }
}
