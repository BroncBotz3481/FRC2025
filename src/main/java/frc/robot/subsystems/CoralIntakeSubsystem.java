package frc.robot.subsystems;


import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Rotations;

import au.grapplerobotics.LaserCan;
import au.grapplerobotics.interfaces.LaserCanInterface.RegionOfInterest;
import au.grapplerobotics.interfaces.LaserCanInterface.TimingBudget;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.sim.SparkAbsoluteEncoderSim;
import com.revrobotics.sim.SparkMaxSim;
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
import edu.wpi.first.math.controller.BangBangController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.WristConstants;
import frc.robot.Constants.WristConstants.RollerConstants;
import frc.robot.HWMap;
import frc.robot.HWMap.Coral;
import frc.robot.Setpoints;
import frc.robot.Setpoints.Wrist;

public class CoralIntakeSubsystem extends SubsystemBase
{

  private final DCMotor m_wristGearbox = DCMotor.getNEO(1);

  private final SparkMax m_wristMotor  = new SparkMax(Coral.coralWristMotorID, MotorType.kBrushless);
  private final SparkMax m_rollerMotor = new SparkMax(Coral.coralRollerMotorID, MotorType.kBrushless);

  private final AbsoluteEncoder           m_wristENcoder2 = m_wristMotor.getAbsoluteEncoder();
  private final SparkClosedLoopController wristController = m_wristMotor.getClosedLoopController();
  private final RelativeEncoder           m_wristEncoder  = m_wristMotor.getEncoder();
  private final AbsoluteEncoder           m_absEncoder    = m_wristMotor.getAbsoluteEncoder();

  // Simulation stuff
  private final DCMotor m_wristMotorGearbox  = DCMotor.getNEO(1);
  private final DCMotor m_rollerMotorGearbox = DCMotor.getNEO(1);

  private final FlywheelSim m_wristSim = new FlywheelSim(
      LinearSystemId.createFlywheelSystem(
          m_wristMotorGearbox,
          WristConstants.kWristMomentOfInertia,
          WristConstants.kWristGearRatio),
      m_wristMotorGearbox,
      1.0 / 4096.0);

  private final FlywheelSim m_rollerSim = new FlywheelSim(LinearSystemId.createFlywheelSystem(
      m_rollerMotorGearbox,
      RollerConstants.kWristMomentOfInertia,
      RollerConstants.kWristGearRatio), m_rollerMotorGearbox, 1.0 / 4096.0);

  private final SparkMaxSim             m_wristMotorSim      = new SparkMaxSim(m_wristMotor, m_wristMotorGearbox);
  private final SparkMaxSim             m_rollerMotorSim     = new SparkMaxSim(m_rollerMotor, m_rollerMotorGearbox);
  private final SparkAbsoluteEncoderSim m_wristAbsEncoderSim = m_wristMotorSim.getAbsoluteEncoderSim();


  private final Mechanism2d         wristMechanism = new Mechanism2d(10, 10);
  private final MechanismRoot2d     wristMech      = wristMechanism.getRoot("Wrist", 5, 0);
  private final MechanismLigament2d wristArm       = wristMech.append(new MechanismLigament2d("Wrist Arm", 4, 90));

  public CoralIntakeSubsystem()
  {
    SparkMaxConfig cfg = new SparkMaxConfig();
    cfg
        .smartCurrentLimit(IntakeConstants.k_wristCurrentLimit) // Move to Constants
        .closedLoopRampRate(IntakeConstants.k_wristClosedLoopRampRate) // Move to Constants
        .idleMode(IdleMode.kCoast)
        .inverted(false)
        .encoder
        .positionConversionFactor(1);

    cfg
        .absoluteEncoder
        .inverted(true)
        .positionConversionFactor(1);
    cfg
        .closedLoop
        .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
        .positionWrappingEnabled(true)
        .positionWrappingInputRange(0, 1)
        .pid(1, 0, 0);
    SparkMaxConfig cfgRoller = new SparkMaxConfig();
    cfgRoller.inverted(true);
    m_rollerMotor.configure(cfgRoller, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    m_wristEncoder.setPosition(0);
    m_wristMotor.configure(cfg, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    SmartDashboard.putData("Wrist", wristMechanism);
    synchronizeEncoders();
    if(RobotBase.isSimulation())
    {
      m_wristAbsEncoderSim.setZeroOffset(0.27);
      m_wristAbsEncoderSim.setPosition(0);
    }
  }


  @Override
  public void simulationPeriodic()
  {
    // In this method, we update our simulation of what our arm is doing
    // First, we set our "inputs" (voltages)
    m_wristSim.setInput(m_wristMotorSim.getAppliedOutput() * RoboRioSim.getVInVoltage());
    m_rollerSim.setInput(m_rollerMotorSim.getAppliedOutput() * RoboRioSim.getVInVoltage());

    // Next, we update it. The standard loop time is 20ms.
    m_wristSim.update(0.02);
    m_rollerSim.update(0.02);

    // Finally, we set our simulated encoder's readings and simulated battery voltage
    //m_encoderSim.setDistance(m_coralArmSim.getAngleRads());
    m_wristMotorSim.iterate(m_wristSim.getAngularVelocityRPM(),
                            RoboRioSim.getVInVoltage(), // Simulated battery voltage, in Volts
                            0.02); // Time interval, in Seconds

    m_wristMotorSim.iterate(m_wristSim.getAngularVelocityRPM(),
                            RoboRioSim.getVInVoltage(), // Simulated battery voltage, in Volts
                            0.02);
    m_rollerMotorSim.iterate(m_rollerSim.getAngularVelocityRPM(),
                             RoboRioSim.getVInVoltage(),
                             // Simulated battery voltage, in Volts
                             0.02);
     m_wristAbsEncoderSim.iterate(m_wristSim.getAngularVelocityRPM(), 0.02);

    // SimBattery estimates loaded battery voltages
    RoboRioSim.setVInVoltage(
        BatterySim.calculateDefaultBatteryLoadedVoltage(
            m_rollerSim.getCurrentDrawAmps() + m_wristSim.getCurrentDrawAmps()));

    // Update the Mechanism Arm angle based on the simulated arm angle
    wristArm.setAngle(Rotations.of(m_wristAbsEncoderSim.getPosition()).in(Degrees));
  }

  public Command spitCoralOut(double speed, double angle)
  {
    return run(() -> {
      wristController.setReference(angle, ControlType.kPosition);
      m_rollerMotor.set(speed);
    });
  }

  BangBangController bangBangController = new BangBangController(0.001);

  public Command setWristAngle(double angle)
  {
    return run(() -> {
      wristController.setReference(angle, ControlType.kPosition);
    });
  }


  public void synchronizeEncoders()
  {
    m_wristEncoder.setPosition(m_wristENcoder2.getPosition() - WristConstants.kWristOffset.in(Rotations));
  }

  public Command setWristPower(double d)
  {
    return run(() -> m_wristMotor.set(d));
  }

  @Override
  public void periodic()
  {
    SmartDashboard.putNumber("Wrist Angle", m_wristEncoder.getPosition());
    SmartDashboard.putNumber("Wrist Angle Absolute Encoder", m_wristENcoder2.getPosition());
  }


  public Command setCoralIntakePower(double i)
  {
    return run(() -> m_rollerMotor.set(i));
  }

  public Trigger atScoringAngle()
  {
    return new Trigger(() -> MathUtil.isNear(Wrist.active, m_absEncoder.getPosition(), 0.01));
  }

  public Command wristScore()
  {
    return setWristAngle(Wrist.active);
  }

  public Command wristIntake()
  {
    return spitCoralOut(-0.1, Wrist.active);
  }

  public Command wristRest()
  {
    return spitCoralOut(0, Wrist.rest);
  }

  public Command wristOuttake()
  {
    return spitCoralOut(1, Wrist.active);
  }

}

