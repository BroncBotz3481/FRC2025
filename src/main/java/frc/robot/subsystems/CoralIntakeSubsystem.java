package frc.robot.subsystems;


import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.IntakeConstants;

public class CoralIntakeSubsystem extends SubsystemBase {

        private final DCMotor m_wristGearbox = DCMotor.getNEO(3);

    private final SparkMax m_wrist = new SparkMax(IntakeConstants.coralWristMotorID, MotorType.kBrushless);
    private final SparkMax m_coralRollers = new SparkMax(IntakeConstants.coralRollerMotorID, MotorType.kBrushless);
    
    private final SparkClosedLoopController rollerController = m_coralRollers.getClosedLoopController();
    private final SparkClosedLoopController wristController = m_wrist.getClosedLoopController();
    private final RelativeEncoder m_encoder = m_wrist.getEncoder();


    public CoralIntakeSubsystem() {
     final SingleJointedArmSim m_wrist =
            new SingleJointedArmSim(
                    m_wristGearbox,
                    IntakeConstants.kIntakeReduction,
                    SingleJointedArmSim.estimateMOI(IntakeConstants.kIntakeLength, ArmConstants.kArmMass),
                    IntakeConstants.kMinAngleRads,
                    IntakeConstants.kMaxAngleRads,
                    IntakeConstants.kIntakeLength,
                    false,
                    0,
                    0.02 / 4096.0,
                    0.0// Add noise with a std-dev of 1 tick
            );
    }


    public Command setCoralIntakeRoller(double speed){
        return run(()->{
            m_coralRollers.set(speed * IntakeConstants.defaultrRollerSpeed);
        });
    }

    public Command setWristAngle(double angle){
        return run(()->{});
    }
}

