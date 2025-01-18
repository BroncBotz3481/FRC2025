package frc.robot.systems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class ScoringSystem {
    public Command scoreCoral(){
        // Arm down, elevator down, drive backwards x in
        return Commands.none();
    }

    public Command scoreAlgaeProcessor(){
        return Commands.none();
    }
    public Command scoreAlgaeNet(){
        return Commands.none();
    }

}
