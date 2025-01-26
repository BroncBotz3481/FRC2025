package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Second;

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
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.hal.SimDevice;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.DIOSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.RobotMath.Arm;



public class CoralArmSubsystem extends SubsystemBase {

    // The arm gearbox represents a gearbox containing two Vex 775pro motors.
    private final DCMotor m_armGearbox = DCMotor.getNEO(2);


    // The P gain for the PID controller that drives this arm.
    private double m_armKp = ArmConstants.kDefaultArmKp;

    private final SparkMax m_motor = new SparkMax(14, MotorType.kBrushless);
    private final SparkMaxSim m_motorSim = new SparkMaxSim(m_motor, m_armGearbox);
    private final SparkClosedLoopController m_controller = m_motor.getClosedLoopController();
    private final RelativeEncoder m_encoder = m_motor.getEncoder();

    private final DigitalInput coralLoaded = new DigitalInput(8); // Digital Input returns true or false
    private final DIOSim coralLoadedSim = new DIOSim(coralLoaded); // Sim Digital Input for robot.
    //sim

    // Sensors
    private final DigitalInput m_limitSwitchHigh    = new DigitalInput(4);
    private       DIOSim       m_limitSwitchHighSim = null;
    private final DigitalInput m_limitSwitchLow     = new DigitalInput(5);
    private       DIOSim       m_limitSwitchLowSim  = null;
    private final DigitalInput m_coralInBin         = new DigitalInput(6);
    private       DIOSim       m_coralInBinSim      = null;
    private final DigitalInput m_coralInArm         = new DigitalInput(7);
    private       DIOSim       m_coralInArmSim      = null;
    // Standard classes for controlling our arm


    // Simulation classes help us simulate what's going on, including gravity.
    // This arm sim represents an arm that can travel from -75 degrees (rotated down front)
    // to 255 degrees (rotated down in the back).
    private final SingleJointedArmSim m_coralArmSim =
            new SingleJointedArmSim(
                    m_armGearbox,
                    ArmConstants.kArmReduction,
                    SingleJointedArmSim.estimateMOI(ArmConstants.kArmLength, ArmConstants.kArmMass),
                    ArmConstants.kArmLength,
                    ArmConstants.kMinAngleRads,
                    ArmConstants.kMaxAngleRads,
                    true,
                    0,
                    0.02 / 4096.0,
                    0.0 // Add noise with a std-dev of 1 tick
            );

    ArmFeedforward armFeedforward = new ArmFeedforward(ArmConstants.kArmkS, ArmConstants.kArmkG, ArmConstants.kArmkV, ArmConstants.kArmkA);


    // Create a Mechanism2d display of an Arm with a fixed ArmTower and moving Arm.
    private final Mechanism2d m_mech2d = new Mechanism2d(60, 60);
    private final MechanismRoot2d m_armPivot = m_mech2d.getRoot("ArmPivot", 30, 30);
    private final MechanismLigament2d m_armTower =
            m_armPivot.append(new MechanismLigament2d("ArmTower", 30, -90));
    private final MechanismLigament2d m_arm =
            m_armPivot.append(
                    new MechanismLigament2d(
                            "Arm",
                            30,
                            Units.radiansToDegrees(m_coralArmSim.getAngleRads()),
                            6,
                            new Color8Bit(Color.kYellow)));


    /**
     * Subsystem constructor.
     */
    public CoralArmSubsystem() {
        SparkMaxConfig config = new SparkMaxConfig();
        config
        .smartCurrentLimit(40)
        .closedLoopRampRate(0.25)
        .closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pid(ArmConstants.kDefaultArmKp, ArmConstants.kArmKi, ArmConstants.kArmKd)
        .outputRange(-1, 1)
        .maxMotion
        .maxVelocity(Arm.convertAngleToSensorUnits(Degrees.of(140)).per(Second).in(RPM))
        .maxAcceleration(Arm.convertAngleToSensorUnits(Degrees.of(180)).per(Second).per(Second).in(RPM.per(Second)))
        .allowedClosedLoopError(Arm.convertAngleToSensorUnits(Degrees.of(1)).in(Rotations));
    m_motor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);

        // Put Mechanism 2d to SmartDashboard
        SmartDashboard.putData("coralArm Sim", m_mech2d);
        m_armTower.setColor(new Color8Bit(Color.kBlue));

    if (RobotBase.isSimulation())
    {
      m_limitSwitchLowSim = new DIOSim(m_limitSwitchLow);
      m_limitSwitchHighSim = new DIOSim(m_limitSwitchHigh);
      m_coralInBinSim = new DIOSim(m_coralInBin);
      m_coralInArmSim = new DIOSim(m_coralInArm);
      SmartDashboard.putData("Coral Arm Limit Switch High", m_limitSwitchHigh);
      SmartDashboard.putData("Coral Arm Limit Switch Low", m_limitSwitchLow);
      SmartDashboard.putData("Coral Arm Coral in Bin", m_coralInBin);
      SmartDashboard.putData("Coral Arm Coral in Arm", m_coralInArm);
    }
    }


    /**
     * Update the simulation model.
     */
    public void simulationPeriodic() {
        // In this method, we update our simulation of what our arm is doing
        // First, we set our "inputs" (voltages)
        m_coralArmSim.setInput(m_motorSim.getAppliedOutput() * RoboRioSim.getVInVoltage());

        // Next, we update it. The standard loop time is 20ms.
        m_coralArmSim.update(0.020);

        // Finally, we set our simulated encoder's readings and simulated battery voltage
        //m_encoderSim.setDistance(m_coralArmSim.getAngleRads());
        m_motorSim.iterate(
            RotationsPerSecond.of(Arm.convertAngleToSensorUnits(Radians.of(m_coralArmSim.getVelocityRadPerSec())).in(Rotations))
                              .in(RPM),
            RoboRioSim.getVInVoltage(), // Simulated battery voltage, in Volts
            0.02); // Time interval, in Seconds
    
        // SimBattery estimates loaded battery voltages
        RoboRioSim.setVInVoltage(
                BatterySim.calculateDefaultBatteryLoadedVoltage(m_coralArmSim.getCurrentDrawAmps()));

        // Update the Mechanism Arm angle based on the simulated arm angle
        m_arm.setAngle(Units.radiansToDegrees(m_coralArmSim.getAngleRads()));

    }


    /**
     * Run the control loop to reach and maintain the setpoint from the preferences.
     */
    public void reachSetpoint(double setPointDegree) {//goal-in degrees?or rad
        m_controller.setReference(Arm.convertAngleToSensorUnits(Degrees.of(setPointDegree)).in(Rotations),
        ControlType.kMAXMotionPositionControl, ClosedLoopSlot.kSlot0, armFeedforward.calculate(m_coralArmSim.getAngleRads(),m_coralArmSim.getVelocityRadPerSec()));
    }

    public double getAngle()
    {
      return Arm.convertSensorUnitsToAngle(Rotations.of(m_encoder.getPosition())).in(Degrees);
    }


    public Command setGoal(double degree) {
        return run(() -> reachSetpoint(degree));
    }


    public Command setCoralArmAngle(double degree) {
        return  setGoal(degree).until(() -> aroundAngle(degree));
    }

    public void stop() {
        m_motor.set(0.0);
    }

    @Override
    public void periodic()
    {
    //    System.out.println(getAngle());
    //    System.out.println(Units.radiansToDegrees(m_coralArmSim.getAngleRads()));
    }
   
    public boolean coralInLoadPosition() {return false;}//Sim

    public boolean coralLoaded() {return coralLoaded.get();}//Sim


    /**
     * Gets the height of the elevator and compares it to the given height with the given tolerance.
     *
     * @param degree         Height in meters
     * @param allowableError Tolerance in meters.
     * @return Within that tolerance.
     */
    public boolean aroundAngle(double degree, double allowableError) {
        return MathUtil.isNear(degree, m_encoder.getPosition(), allowableError);
    }

    public boolean aroundAngle(double degree) {
        return aroundAngle(degree, ArmConstants.kAngleAllowableError);
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
