package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Minute;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.AbsoluteEncoder;
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
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.DIOSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants;
import frc.robot.Constants.AlgaeArmConstants;
import frc.robot.RobotMath.AlgaeArm;


public class AlgaeArmSubsystem extends SubsystemBase
{

  public final Trigger atMin
      = new Trigger(() -> getAngle().lte(AlgaeArmConstants.kAlgaeArmMinAngle.plus(Degrees.of(5))));
  public final Trigger atMax
      = new Trigger(() -> getAngle().gte(AlgaeArmConstants.kAlgaeArmMaxAngle.minus(Degrees.of(5))));

  // The arm gearbox represents a gearbox containing two Vex 775pro motors.
  private final DCMotor                   m_armGearbox = DCMotor.getNEO(1);
  private final SparkMax                  m_motor      = new SparkMax(AlgaeArmConstants.algaeArmMotorID,
                                                                      MotorType.kBrushless);
  private       DigitalInput        armLoaded              = new DigitalInput(1);
  private       DigitalInput        armInLoadedPosition    = new DigitalInput(2);

  private final SparkClosedLoopController m_controller = m_motor.getClosedLoopController();
  private final RelativeEncoder           m_encoder    = m_motor.getEncoder();

  // SysId Routine and seutp
  // Mutable holder for unit-safe linear velocity values, persisted to avoid reallocation.
  private final MutAngularVelocity    m_velocity       = RPM.mutable(0);
  // Mutable holder for unit-safe voltage values, persisted to avoid reallocation.
  private final MutVoltage            m_appliedVoltage = Volts.mutable(0);
  // Mutable holder for unit-safe linear distance values, persisted to avoid reallocation.
  private final MutAngle              m_angle          = Rotations.mutable(0);
  // SysID Routine
  private final SysIdRoutine          m_sysIdRoutine   =
      new SysIdRoutine(
          // Empty config defaults to 1 volt/second ramp rate and 7 volt step voltage.
          new SysIdRoutine.Config(Volts.per(Second).of(AlgaeArmConstants.kAlgaeArmRampRate),
                                  Volts.of(1),
                                  Seconds.of(30)),
          new SysIdRoutine.Mechanism(
              // Tell SysId how to plumb the driving voltage to the motor(s).
              m_motor::setVoltage,
              // Tell SysId how to record a frame of data for each motor on the mechanism being
              // characterized.
              log -> {
                // Record a frame for the shooter motor.
                log.motor("arm")
                   .voltage(
                       m_appliedVoltage.mut_replace(m_motor.getAppliedOutput() *
                                                    RobotController.getBatteryVoltage(), Volts))
                   .angularPosition(m_angle.mut_replace(m_encoder.getPosition(), Rotations))
                   .angularVelocity(m_velocity.mut_replace(m_encoder.getVelocity(), RPM));
//                .angularPosition(m_angle.mut_replace(getAngle()))
//                .angularVelocity(m_velocity.mut_replace(getVelocity()));
              },
              this));
  private final AbsoluteEncoder       m_absEncoder     = m_motor.getAbsoluteEncoder();
  // Standard classes for controlling our arm
  private final ProfiledPIDController m_pidController;
  private final ArmFeedforward        m_feedforward    = new ArmFeedforward(AlgaeArmConstants.kAlgaeArmkS,
                                                                            AlgaeArmConstants.kAlgaeArmkG,
                                                                            AlgaeArmConstants.kAlgaeArmKv,
                                                                            AlgaeArmConstants.kAlgaeArmKa);


  // Simulation classes help us simulate what's going on, including gravity.
  // This arm sim represents an arm that can travel from -75 degrees (rotated down front)
  // to 255 degrees (rotated down in the back).
  private final SingleJointedArmSim m_armSim               =
      new SingleJointedArmSim(
          m_armGearbox,
          AlgaeArmConstants.kAlgaeArmReduction,
          SingleJointedArmSim.estimateMOI(AlgaeArmConstants.kAlgaeArmLength, AlgaeArmConstants.kAlgaeArmMass),
          AlgaeArmConstants.kAlgaeArmLength,
          AlgaeArmConstants.kAlgaeArmMinAngle.in(Radians),
          AlgaeArmConstants.kAlgaeArmMaxAngle.in(Radians),
          true,
          AlgaeArmConstants.kAlgaeArmStartingAngle.in(Radians),
          0.02 / 4096.0,
          0.0 // Add noise with a std-dev of 1 tick
      );
  private final SparkMaxSim         m_motorSim             = new SparkMaxSim(m_motor, m_armGearbox);
  private       DIOSim              armLoadedSim           = new DIOSim(armLoaded);
  private       DIOSim              armInLoadedPositionSim = new DIOSim(armInLoadedPosition);


  public AlgaeArmSubsystem()
  {
    SparkMaxConfig config = new SparkMaxConfig();
    config
        .smartCurrentLimit(AlgaeArmConstants.kAlgaeArmStallCurrentLimitAmps)
        .closedLoopRampRate(AlgaeArmConstants.kAlgaeArmRampRate)
        .idleMode(IdleMode.kBrake)
        .inverted(AlgaeArmConstants.kAlgaeArmInverted)
        .closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pid(AlgaeArmConstants.kAlgaeArmKp, AlgaeArmConstants.kAlgaeArmKi, AlgaeArmConstants.kAlgaeArmKd)
        .outputRange(-1, 1)
        .maxMotion
        .maxVelocity(AlgaeArmConstants.kAlgaeArmMaxVelocityRPM)
        .maxAcceleration(AlgaeArmConstants.kAlgaeArmMaxAccelerationRPMperSecond)
        .allowedClosedLoopError(AlgaeArmConstants.kAlgaeArmAllowedClosedLoopError.in(Rotations));
    m_motor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    synchronizeAbsoluteEncoder();

    // PID Controller
    m_pidController = new ProfiledPIDController(AlgaeArmConstants.kAlgaeArmKp,
                                                AlgaeArmConstants.kAlgaeArmKi,
                                                AlgaeArmConstants.kAlgaeArmKd,
                                                new Constraints(AlgaeArmConstants.kAlgaeArmMaxVelocityRPM,
                                                                AlgaeArmConstants.kAlgaeArmMaxAccelerationRPMperSecond));
    m_pidController.setTolerance(0.01);


  }


  /**
   * Update the simulation model.
   */
  public void simulationPeriodic()
  {
    // In this method, we update our simulation of what our arm is doing
    // First, we set our "inputs" (voltages)
    m_armSim.setInput(m_motorSim.getAppliedOutput() * RoboRioSim.getVInVoltage());

    // Next, we update it. The standard loop time is 20ms.
    m_armSim.update(0.020);

    m_motorSim.iterate(
        RotationsPerSecond.of(AlgaeArm.convertAlgaeAngleToSensorUnits(Radians.of(m_armSim.getVelocityRadPerSec()))
                                      .in(Rotations))
                          .in(RPM),
        RoboRioSim.getVInVoltage(), // Simulated battery voltage, in Volts
        0.02); // Time interval, in Seconds
    // Finally, we set our simulated encoder's readings and simulated battery voltage
    m_encoder.setPosition(AlgaeArm.convertAlgaeAngleToSensorUnits(Radians.of(m_armSim.getAngleRads())).in(Rotations));

    // SimBattery estimates loaded battery voltages
    RoboRioSim.setVInVoltage(
        BatterySim.calculateDefaultBatteryLoadedVoltage(m_armSim.getCurrentDrawAmps()));

    // Update the Mechanism Arm angle based on the simulated arm angle
    Constants.kAlgaeArmMech.setAngle(Units.radiansToDegrees(m_armSim.getAngleRads()));

  }


  /**
   * Near the maximum Angle of the arm within X degrees.
   *
   * @param toleranceDegrees Degrees close to maximum of the Arm.
   * @return is near the maximum of the arm.
   */
  public boolean nearMax(double toleranceDegrees)
  {
    if (getAngle().isNear(AlgaeArmConstants.kAlgaeArmMaxAngle, Degrees.of(toleranceDegrees)))
    {
      System.out.println("Current angle: " + getAngle().in(Degrees));
      System.out.println(
          "At max:" + getAngle().isNear(AlgaeArmConstants.kAlgaeArmMaxAngle, Degrees.of(toleranceDegrees)));
    }
    return getAngle().isNear(AlgaeArmConstants.kAlgaeArmMaxAngle, Degrees.of(toleranceDegrees));

  }

  /**
   * Near the minimum angle of the Arm in within X degrees.
   *
   * @param toleranceDegrees Tolerance of the Arm.
   * @return is near the minimum of the arm.
   */
  public boolean nearMin(double toleranceDegrees)
  {
    if (getAngle().isNear(AlgaeArmConstants.kAlgaeArmMinAngle, Degrees.of(toleranceDegrees)))
    {
      System.out.println("Current angle: " + getAngle().in(Degrees));
      System.out.println(
          "At min:" + getAngle().isNear(AlgaeArmConstants.kAlgaeArmMinAngle, Degrees.of(toleranceDegrees)));
    }
    return getAngle().isNear(AlgaeArmConstants.kAlgaeArmMinAngle, Degrees.of(toleranceDegrees));

  }

  /**
   * Synchronizes the NEO encoder with the attached Absolute Encoder.
   */
  public void synchronizeAbsoluteEncoder()
  {
    m_encoder.setPosition(Rotations.of(m_absEncoder.getPosition())
                                   .minus(AlgaeArmConstants.kAlgaeArmOffsetToHorizantalZero)
                                   .in(Rotations));
  }

  /**
   * Runs the SysId routine to tune the Arm
   *
   * @return SysId Routine command
   */
  public Command runSysIdRoutine()
  {
    return m_sysIdRoutine.dynamic(Direction.kForward).until(atMax)
                         .andThen(m_sysIdRoutine.dynamic(Direction.kReverse).until(atMin))
                         .andThen(m_sysIdRoutine.quasistatic(Direction.kForward).until(atMax))
                         .andThen(m_sysIdRoutine.quasistatic(Direction.kReverse).until(atMin));
  }


  public void reachSetpoint(double setPointDegree)
  {
    double  goalPosition = AlgaeArm.convertAlgaeAngleToSensorUnits(Degrees.of(setPointDegree)).in(Rotations);
    boolean rioPID       = true;
    if (rioPID)
    {
      double pidOutput     = m_pidController.calculate(m_encoder.getPosition(), goalPosition);
      State  setpointState = m_pidController.getSetpoint();
      m_motor.setVoltage(pidOutput +
                         m_feedforward.calculate(setpointState.position,
                                                 setpointState.velocity)
                        );
    } else
    {
      m_controller.setReference(goalPosition,
                                ControlType.kMAXMotionPositionControl, ClosedLoopSlot.kSlot0);
    }
  }

  /**
   * Get the Angle of the Arm.
   *
   * @return Angle of the Arm.
   */
  public Angle getAngle()
  {
    m_angle.mut_replace(AlgaeArm.convertSensorUnitsToAlgaeAngle(m_angle.mut_replace(m_encoder.getPosition(),
                                                                                    Rotations)));
    return m_angle;
  }

  /**
   * Get the velocity of Arm.
   *
   * @return Velocity of the Arm.
   */
  public AngularVelocity getVelocity()
  {
    return m_velocity.mut_replace(AlgaeArm.convertAlgaeAngleToSensorUnits(Rotations.of(m_encoder.getVelocity()))
                                          .per(Minute));
  }

  public Command setGoal(double degree)
  {
    return run(() -> reachSetpoint(degree));
  }

  public Command setAlgaeArmAngle(double degree)
  {
    return setGoal(degree).until(() -> aroundAngle(degree));
  }


  public void stop()
  {
    m_motor.set(0.0);
  }

  @Override
  public void periodic()
  {
    //    System.out.println(getAngle());
    //    System.out.println(Units.radiansToDegrees(m_AlgaeArmSim.getAngleRads()));
  }

  public boolean algaeInLoadPosition()
  {
    System.out.println(armInLoadedPosition.get());
    return armInLoadedPosition.get();//m_algaeInArm.get()&&aroundAngle(135);//only check the angle-still need check elev?
  }

  public boolean algaeLoaded()
  {
    return armLoaded.get();//m_algaeInBin.get()|| m_algaeInArm.get();
  }

  public boolean aroundAngle(double degree, double allowableError)
  {
    //get current angle compare to aimed angle
    return MathUtil.isNear(degree, getAngle().in(Degrees), allowableError);
  }

  public boolean aroundAngle(double degree)
  {
    return aroundAngle(degree, AlgaeArmConstants.kAlgaeAngleAllowableError);
  }




  /*
  public void close() {
    m_motor.close();
    m_encoder.close();
    m_mech2d.close();
    m_armPivot.close();
    m_controller.close();
    m_arm.close();
  }*/

}
