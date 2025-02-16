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
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.RobotState;
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


  // The arm gearbox represents a gearbox containing two Vex 775pro motors.
  private final DCMotor                   m_armGearbox = DCMotor.getNEO(1);
  private final SparkMax                  m_motor      = new SparkMax(AlgaeArmConstants.algaeArmMotorID,
                                                                      MotorType.kBrushless);
  private final RelativeEncoder           m_encoder    = m_motor.getEncoder();
  private final AbsoluteEncoder       m_absEncoder     = m_motor.getAbsoluteEncoder();
  // Standard classes for controlling our arm
  private final ProfiledPIDController m_pidController = new ProfiledPIDController(AlgaeArmConstants.kAlgaeArmKp,
          AlgaeArmConstants.kAlgaeArmKi,
          AlgaeArmConstants.kAlgaeArmKd,
          new Constraints(AlgaeArmConstants.kAlgaeArmMaxVelocityRPM,
                  AlgaeArmConstants.kAlgaeArmMaxAccelerationRPMperSecond));
  private final ArmFeedforward        m_feedforward    = new ArmFeedforward(AlgaeArmConstants.kAlgaeArmkS,
                                                                            AlgaeArmConstants.kAlgaeArmkG,
                                                                            AlgaeArmConstants.kAlgaeArmKv,
                                                                            AlgaeArmConstants.kAlgaeArmKa);

  private final SparkMaxSim         m_motorSim             = new SparkMaxSim(m_motor, m_armGearbox);

  // Simulation classes help us simulate what's going on, including gravity.
  // This arm sim represents an arm that can travel from -75 degrees (rotated down front)
  // to 255 degrees (rotated down in the back).
  private       SingleJointedArmSim m_armSim               ;
  private       DigitalInput        armLoaded              = new DigitalInput(2);
  private       DigitalInput        armInLoadedPosition    = new DigitalInput(1);
  private       DIOSim              armLoadedSim           = new DIOSim(armLoaded);
  private       DIOSim              armInLoadedPositionSim = new DIOSim(armInLoadedPosition);


  public AlgaeArmSubsystem()
  {
    SparkMaxConfig config = new SparkMaxConfig();
    config
        .smartCurrentLimit(AlgaeArmConstants.kAlgaeArmStallCurrentLimitAmps)
        .openLoopRampRate(AlgaeArmConstants.kAlgaeArmRampRate)//?
        .idleMode(IdleMode.kBrake)//lock at that angle  coast-keep going
        .inverted(AlgaeArmConstants.kAlgaeArmInverted)
        .closedLoop//?
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .maxMotion//need to also make sure not hitting current limits
        .maxVelocity(AlgaeArmConstants.kAlgaeArmMaxVelocityRPM)//for the encoder-unit conversion
        .maxAcceleration(AlgaeArmConstants.kAlgaeArmMaxAccelerationRPMperSecond)
        .allowedClosedLoopError(AlgaeArmConstants.kAlgaeArmAllowedClosedLoopError.in(Rotations));
    m_motor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);

    if(RobotBase.isSimulation()){
      m_armSim = new SingleJointedArmSim(
              m_armGearbox,
              AlgaeArmConstants.kAlgaeArmReduction,
              SingleJointedArmSim.estimateMOI(AlgaeArmConstants.kAlgaeArmLength, AlgaeArmConstants.kAlgaeArmMass),
              AlgaeArmConstants.kAlgaeArmLength,
              AlgaeArmConstants.kAlgaeArmMinAngle.in(Radians),
              AlgaeArmConstants.kAlgaeArmMaxAngle.in(Radians),
              true,
              AlgaeArmConstants.kAlgaeArmStartingAngle.in(Radians),
              0.02 / 4096.0,//encoder resolution
              0.0); // Add noise with a std-dev of 1 tick
    }

    synchronizeAbsoluteEncoder();

    //m_pidController.setTolerance(0.01);


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
    //update relative encoder

    // SimBattery estimates loaded battery voltages
    RoboRioSim.setVInVoltage(
        BatterySim.calculateDefaultBatteryLoadedVoltage(m_armSim.getCurrentDrawAmps()));

    // Update the Mechanism Arm angle based on the simulated arm angle
    Constants.armMech.setAngle(Units.radiansToDegrees(m_armSim.getAngleRads()));

  }

  /**
   * Synchronizes the NEO encoder with the attached Absolute Encoder.
   */
  public void synchronizeAbsoluteEncoder()
  {
    m_encoder.setPosition(Rotations.of(m_absEncoder.getPosition())
                                   .minus(AlgaeArmConstants.kAlgaeArmOffsetToHorizantalZero)
                                   .in(Rotations));//update during simulation or just intialize in the constructor
  }




  public void reachSetpoint(double setPointDegree)
  {
    double  goalPosition = AlgaeArm.convertAlgaeAngleToSensorUnits(Degrees.of(setPointDegree)).in(Rotations);
    /*boolean rioPID       = true;//?
    if (rioPID)
    {*/
      double pidOutput     = m_pidController.calculate(m_encoder.getPosition(), goalPosition);
      State  setpointState = m_pidController.getSetpoint();
      m_motor.setVoltage(pidOutput +
                         m_feedforward.calculate(setpointState.position,
                                                 setpointState.velocity)//data related to setpoint
                        );//constant velcity 0 acceleration

  }

  /**
   * Get the Angle of the Arm.
   *
   * @return Angle of the Arm.
   */
  public Angle getAngle()//cuz sometimes we need to convert to degrees so use Angle instead of double
  {
    return AlgaeArm.convertSensorUnitsToAlgaeAngle(Rotations.of(m_encoder.getPosition()));
   // m_angle.mut_replace(AlgaeArm.convertSensorUnitsToAlgaeAngle(m_angle.mut_replace(m_encoder.getPosition(),
      //                                                                              Rotations)));

  }

  /**
   * Get the velocity of Arm.
   *
   * @return Velocity of the Arm.
   */
  public AngularVelocity getVelocity()
  {
    return AlgaeArm.convertSensorUnitsToAlgaeAngle(Rotations.of(m_encoder.getVelocity())).per(Minute);//m_velocity.mut_replace(AlgaeArm.convertAlgaeAngleToSensorUnits(Rotations.of(m_encoder.getVelocity()))
                                    //      .per(Minute));
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
    return MathUtil.isNear(degree, getAngle().in(Degrees), allowableError);//?
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
