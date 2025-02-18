package frc.robot.subsystems;


import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.DIOSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;
import frc.robot.RobotMath;
import frc.robot.RobotMath.Arm;

import static edu.wpi.first.units.Units.*;

public class ArmSubsystem extends SubsystemBase {
    //setup
    private final DCMotor m_armGearbox = DCMotor.getNEO(1);
    private final SparkMax m_motor = new SparkMax(ArmConstants.armMotorID, MotorType.kBrushless);
    private final SparkMaxSim m_motorSim = new SparkMaxSim(m_motor, m_armGearbox);
    private final RelativeEncoder m_encoder = m_motor.getEncoder();
    private final AbsoluteEncoder m_absEncoder = m_motor.getAbsoluteEncoder();
    private final ArmFeedforward m_armFeedForward = new ArmFeedforward(ArmConstants.kArmkS,
            ArmConstants.kArmkG,
            ArmConstants.kArmkV,
            ArmConstants.kArmkA);
    private final ProfiledPIDController m_controller = new ProfiledPIDController(ArmConstants.kArmkP,
            ArmConstants.kArmkI,
            ArmConstants.kArmkD,
            new Constraints(ArmConstants.kArmMaxVelocity,
                    ArmConstants.kArmMaxAcceleration));
    private SingleJointedArmSim m_armSim;
    //sensors
    private DigitalInput armInLoadPosition = new DigitalInput(1);
    private DigitalInput armLoaded = new DigitalInput(2);
    private DIOSim armInLoadPositionSim = new DIOSim(armInLoadPosition);
    private DIOSim armLoadedSim = new DIOSim(armLoaded);

    //constructor
    public ArmSubsystem() {
        SparkMaxConfig config = new SparkMaxConfig();
        config.smartCurrentLimit(ArmConstants.ArmStallCurrentLimit)
                .openLoopRampRate(ArmConstants.ArmRampRate)
                .idleMode(IdleMode.kBrake)
                .inverted(ArmConstants.ArmInverted)
                .closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .maxMotion
                .maxVelocity(ArmConstants.kArmMaxVelocity)
                .maxAcceleration(ArmConstants.kArmMaxAcceleration)
                .allowedClosedLoopError(ArmConstants.kArmAllowedClosedLoopError);
        m_motor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);

        if (RobotBase.isSimulation()) {
            m_armSim = new SingleJointedArmSim(m_armGearbox,
                    ArmConstants.kArmReduction,
                    SingleJointedArmSim.estimateMOI(ArmConstants.kArmLength, ArmConstants.kArmMass),
                    ArmConstants.kArmLength,
                    ArmConstants.kArmMinAngle.in(Radians),
                    ArmConstants.kArmMaxAngle.in(Radians),
                    true,
                    ArmConstants.kArmStartingAngle.in(Radians),
                    0.02 / 4096.0,
                    0.0);
        }
        synchronizeAbsoluteEncoder();
    }

    //synchronize the NEO encoder with the attached absolute encoder
    public void synchronizeAbsoluteEncoder() {
        m_encoder.setPosition(Rotations.of(m_absEncoder.getPosition())
                .minus(ArmConstants.kArmOffsetToHorizontalZero)
                .in(Rotations));
    }


    //simulationPeriodic
    public void simulationPeriodic() {
        m_armSim.setInput(m_motorSim.getAppliedOutput() * RoboRioSim.getVInVoltage());

        m_armSim.update(0.02);


        m_motorSim.iterate(RotationsPerSecond.of(Arm.convertArmAngleToSensorUnits(Radians.of(m_armSim.getVelocityRadPerSec()))
                        .in(Rotations)).in(RPM),
                        RoboRioSim.getVInVoltage(),
                        0.02);

        RoboRioSim.setVInVoltage(BatterySim.calculateDefaultBatteryLoadedVoltage(m_armSim.getCurrentDrawAmps()));

        m_encoder.setPosition(Arm.convertArmAngleToSensorUnits(Radians.of(m_armSim.getAngleRads())).in(Rotations));//?


        Constants.armMech.setAngle(Units.radiansToDegrees(m_armSim.getAngleRads()));
    }

    //methods/command
    public Angle getAngle() {
        return Arm.convertSensorUnitsToArmAngle(Rotations.of(m_encoder.getPosition()));
    }

    public AngularVelocity getVelocity() {
        return Arm.convertSensorUnitsToArmAngle(Rotations.of(m_encoder.getVelocity())).per(Minute);
    }

    public void reachGoal(double goalDegrees) {
        double goal = Arm.convertArmAngleToSensorUnits(Degrees.of(goalDegrees)).in(Rotations);

        m_motor.setVoltage(m_armFeedForward.calculate(m_controller.getSetpoint().position, m_controller.getSetpoint().velocity)
                + m_controller.calculate(m_encoder.getPosition(), goal));
    }

    public Command setGoal(double goalDegrees) {
        return run(() -> reachGoal(goalDegrees));
    }

    public Command setArmAngle(double goalDegrees) {
        return setGoal(goalDegrees).until(() -> aroundAngle(goalDegrees));
    }

   public boolean aroundAngle(double degrees){
        return aroundAngle(degrees, ArmConstants.kArmDefaultTolerance);
   }

   public boolean aroundAngle(double degrees, double tolerance){
        return MathUtil.isNear(degrees, getAngle().in(Degrees),tolerance);
   }

   public boolean armInLoadPosition(){
        return armInLoadPosition.get();
   }

   public boolean armLoaded(){
        return armLoaded.get();
   }
    public void stop()
    {
        m_motor.set(0.0);
    }

    @Override
    public void periodic()
    {
        //    System.out.println(getAngle());
    }

}

