// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.MAXMotionConfig.MAXMotionPositionMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.Constants.ElevatorConstants;

import java.util.function.Supplier;

public class ElevatorSubsystem extends SubsystemBase {

    // This gearbox represents a gearbox containing 1 Neo
    private final DCMotor m_elevatorGearbox = DCMotor.getNEO(1);

    // Standard classes for controlling our elevator
    ElevatorFeedforward m_feedforward =
            new ElevatorFeedforward(
                    ElevatorConstants.kElevatorkS,
                    ElevatorConstants.kElevatorkG,
                    ElevatorConstants.kElevatorkV,
                    ElevatorConstants.kElevatorkA);
    private final SparkMax m_motor = new SparkMax(13, MotorType.kBrushless);
    private final SparkClosedLoopController m_controller = m_motor.getClosedLoopController();
    private final RelativeEncoder m_encoder = m_motor.getEncoder();
    private final SparkMaxSim m_motorSim = new SparkMaxSim(m_motor, m_elevatorGearbox);

    // Simulation classes help us simulate what's going on, including gravity.
    private final ElevatorSim m_elevatorSim =
            new ElevatorSim(
                    m_elevatorGearbox,
                    ElevatorConstants.kElevatorGearing,
                    ElevatorConstants.kCarriageMass,
                    ElevatorConstants.kElevatorDrumRadius,
                    ElevatorConstants.kMinElevatorHeightMeters,
                    ElevatorConstants.kMaxElevatorHeightMeters,
                    true,
                    0,
                    0.01,
                    0.0);

    // Create a Mechanism2d visualization of the elevator
    private final Mechanism2d m_mech2d = new Mechanism2d(20, 12);//height of the barge. currently in meters change that
    private final MechanismRoot2d m_mech2dRoot = m_mech2d.getRoot("Elevator Root", 10, 0);
    private final MechanismLigament2d m_elevatorMech2d =
            m_mech2dRoot.append(
                    new MechanismLigament2d("Elevator", m_elevatorSim.getPositionMeters(), 90));

    /**
     * Subsystem constructor.
     */
    public ElevatorSubsystem() {
        SparkMaxConfig config = new SparkMaxConfig();
        config.encoder
                .positionConversionFactor(ElevatorConstants.kRotaionToMeters) // Converts Rotations to Meters
                .velocityConversionFactor(ElevatorConstants.kRPMtoMPS); // Converts RPM to MPS
        config.closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .pid(ElevatorConstants.kElevatorKp, ElevatorConstants.kElevatorKi, ElevatorConstants.kElevatorKd)
                .maxMotion
                .maxVelocity(ElevatorConstants.kElevatorMaxVelocity)
                .maxAcceleration(ElevatorConstants.kElevatorMaxAcceleration)
                .positionMode(MAXMotionPositionMode.kMAXMotionTrapezoidal)
                .allowedClosedLoopError(0.01);
        m_motor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);

        // Publish Mechanism2d to SmartDashboard
        // To view the Elevator visualization, select Network Tables -> SmartDashboard -> Elevator Sim
        SmartDashboard.putData("Elevator Sim", m_mech2d);
    }

    /**
     * Advance the simulation.
     */
    public void simulationPeriodic() {
        // In this method, we update our simulation of what our elevator is doing
        // First, we set our "inputs" (voltages)
        m_elevatorSim.setInput(m_motorSim.getAppliedOutput() * RoboRioSim.getVInVoltage());

        // Next, we update it. The standard loop time is 20ms.
        m_elevatorSim.update(0.020);

        // Finally, we set our simulated encoder's readings and simulated battery voltage
        m_motorSim.iterate(m_elevatorSim.getVelocityMetersPerSecond(), RoboRioSim.getVInVoltage(), 0.020);

        // SimBattery estimates loaded battery voltages
        RoboRioSim.setVInVoltage(
                BatterySim.calculateDefaultBatteryLoadedVoltage(m_elevatorSim.getCurrentDrawAmps()));
    }

    /**
     * Run control loop to reach and maintain goal.
     *
     * @param goal the position to maintain
     */
    public void reachGoal(double goal) {
        m_controller.setReference(goal,
                ControlType.kPosition,
                ClosedLoopSlot.kSlot0,
                m_feedforward.calculate(m_encoder.getVelocity()));
    }


    /**
     * Get the height in meters.
     *
     * @return Height in meters
     */
    public double getHeight() {
        return m_encoder.getPosition();
    }

    /**
     * A trigger for when the height is at an acceptable tolerance.
     *
     * @param height    Height in Meters
     * @param tolerance Tolerance in meters.
     * @return {@link Trigger}
     */
    public Trigger atHeight(double height, double tolerance) {
        return new Trigger(() -> MathUtil.isNear(height,
                getHeight(),
                tolerance));
    }

    /**
     * Set the goal of the elevator
     *
     * @param goal Goal in meters
     * @return {@link edu.wpi.first.wpilibj2.command.Command}
     */
    public Command setGoal(double goal) {
        return run(() -> reachGoal(goal));
    }


    /**
     * Set the elevator goal and stop when it reaches its target.
     *
     * @param height Height in meters.
     * @return Command which ends when the elevator is near the target height.
     */
    public Command setElevatorHeight(double height) {
        return setGoal(height).until(() -> aroundHeight(height));
    }

    /**
     * Stop the control loop and motor output.
     */
    public void stop() {
        m_motor.set(0.0);
    }

    /**
     * Update telemetry, including the mechanism visualization.
     */
    public void updateTelemetry() {
        // Update elevator visualization with position
        m_elevatorMech2d.setLength(RobotBase.isSimulation() ? m_elevatorSim.getPositionMeters() : m_encoder.getPosition());
    }

    @Override
    public void periodic() {
        updateTelemetry();
    }

    /**
     * Gets the height of the elevator and compares it to the given height with the given tolerance.
     *
     * @param height         Height in meters
     * @param allowableError Tolerance in meters.
     * @return Within that tolerance.
     */
    public boolean aroundHeight(double height, double allowableError) {
        return MathUtil.isNear(height, m_encoder.getPosition(), allowableError);
    }

    /**
     * Gets the height of the elevator and compares it to the given height with the given tolerance.
     *
     * @param height         Height in meters
     * @return Within that tolerance.
     */
    public boolean aroundHeight(double height) {
        return aroundHeight(height, Units.inchesToMeters(ElevatorConstants.kElevatorAllowableError));
    }


}