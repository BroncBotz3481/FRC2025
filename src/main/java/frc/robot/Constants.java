// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Second;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.RobotMath.AlgaeArm;
import frc.robot.RobotMath.CoralArm;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean constants. This
 * class should not be used for any other purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */

public final class Constants
{

  public static final Mechanism2d         sideRobotView = new Mechanism2d(AlgaeArmConstants.kAlgaeArmLength * 2,
                                                                          ElevatorConstants.kMaxElevatorHeight.in(Meters) +
                                                                          AlgaeArmConstants.kAlgaeArmLength +
                                                                          ElevatorConstants.kElevatorUnextendedHeight);
  public static final MechanismRoot2d     kElevatorCarriage;
  public static final MechanismRoot2d     kElevatorJoint; // For fixed elevator
  public static final MechanismLigament2d kAlgaeArmMech;
  public static final MechanismLigament2d kCoralArmMech;
  public static final MechanismLigament2d kElevatorTower;
  public static final MechanismLigament2d kElevatorFixed;
  public static final double              maxSpeed      = 7; // Might need to be 12.7474621

  static
  {
    kElevatorCarriage = Constants.sideRobotView.getRoot("ElevatorCarriage",
                                                        AlgaeArmConstants.kAlgaeArmLength,
                                                        ElevatorConstants.kStartingHeightSim.in(Meters) +
                                                        ElevatorConstants.kElevatorUnextendedHeight);//The pivot
    kAlgaeArmMech = kElevatorCarriage.append(
        new MechanismLigament2d(
            "AlgaeArm",
            AlgaeArmConstants.kAlgaeArmLength,
            AlgaeArmConstants.kAlgaeArmStartingAngle.in(Degrees),
            6,
            new Color8Bit(Color.kGreen)));
    kCoralArmMech = kElevatorCarriage.append(
        new MechanismLigament2d(
            "CoraleArm",
            CoralArmConstants.kCoralArmLength,
            CoralArmConstants.kCoralArmStartingAngle.in(Degrees),
            6,
            new Color8Bit(Color.kPurple)));

    kElevatorTower = kElevatorCarriage.append(new MechanismLigament2d(
        "Elevator",
        ElevatorConstants.kStartingHeightSim.in(Meters),
        -90,
        6,
        new Color8Bit(Color.kRed)));

    kElevatorJoint = Constants.sideRobotView.getRoot("ElevatorJoint",
                                                     AlgaeArmConstants.kAlgaeArmLength,
                                                     ElevatorConstants.kElevatorUnextendedHeight);

    kElevatorFixed = kElevatorJoint.append(new MechanismLigament2d("ElevatorFixed",
                                                                   ElevatorConstants.kElevatorUnextendedHeight,
                                                                   -90,
                                                                   6,
                                                                   new Color8Bit(Color.kPaleVioletRed)));
  }

  public static class OperatorConstants
  {

    public static final int    kDriverControllerPort    = 0;
    public static final int    kOperatorControllerPort  = 4;
    
    public static final double DEADBAND                 = 0.05;
  }

  public static class WristConstants
  {

    public static final double kWristMomentOfInertia = 0.00032; // kg * m^2
    public static final double kWristGearRatio       = (30.0 / 54.0) * 28.0;
    public static final Angle  kWristOffset          = Degrees.of(0.35);

    public static class RollerConstants
    {

      public static final double kWristMomentOfInertia = 0.00032; // kg * m^2
      public static final double kWristGearRatio       = 1.0;
    }
  }

  public static class AlgaeArmConstants
  {

    // The P gain for the PID controller that drives this arm.
    public static final double  kAlgaeArmKp                     = 0.60439;
    public static final double  kAlgaeArmKi                     = 0;
    public static final double  kAlgaeArmKd                     = 0.03164;
    public static final double  kAlgaeArmkS                     = 0.31986; // volts (V)
    public static final double  kAlgaeArmKv                     = 0.00091824; // volts per velocity (V/RPM)
    public static final double  kAlgaeArmKa                     = 0.00023069; // volts per acceleration (V/(RPM/s))
    public static final double  kAlgaeArmkG                     = 0.16271; // volts (V)
    public static final double  kAlgaeArmReduction              = 112.0;
    public static final double  kAlgaeArmMass                   = Units.lbsToKilograms(15); // Kilograms
    public static final double  kAlgaeArmLength                 = Inches.of(31).in(Meters);//.7meter
    public static final Angle   kAlgaeArmStartingAngle          = Degrees.of(-60);
    public static final Angle   kAlgaeArmMinAngle               = Degrees.of(-60);
    public static final Angle   kAlgaeArmMaxAngle               = Degrees.of(215);
    public static final double  kAlgaeArmRampRate               = 0.5;
    public static final Angle   kAlgaeArmOffsetToHorizantalZero = Degrees.of(115.2);
    public static final boolean kAlgaeArmInverted               = false;
    public static final double  kAlgaeArmMaxVelocityRPM         = AlgaeArm.convertAlgaeAngleToSensorUnits(
        Degrees.of(20)).per(
        Second).in(RPM);
    public static final double  kAlgaeArmMaxAccelerationRPMperSecond
                                                                = AlgaeArm.convertAlgaeAngleToSensorUnits(Degrees.of(
                                                                              5)).per(
                                                                              Second).per(Second)
                                                                          .in(RPM.per(Second));
    public static final int     kAlgaeArmStallCurrentLimitAmps  = 40;

    public static final double kAlgaeAngleAllowableError = RobotBase.isSimulation() ? 0.001 : 3;//degree, for testing whether it's aroundAngle

  }

  public static class CoralArmConstants
  {

    // The P gain for the PID controller that drives this arm.
    public static final double kCoralArmKp               = 0.64152;
    public static final double kCoralArmKi               = 0;
    public static final double kCoralArmKd               = 0.08863;
    public static final double kCoralArmkS               = 0.19214; // volts (V)
    public static final double kCoralArmKv               = 0.11319; // volts per velocity (V/RPM)
    public static final double kCoralArmKa               = 0; // volts per acceleration (V/(RPM/s))
    public static final double kCoralArmkG               = 0.023981; // volts (V)
    public static final double kCoralAngleAllowableError = RobotBase.isSimulation() ? 5 : 3;
//degree, for testing whether it's aroundAngle

    public static final double  kCoralArmReduction              = 112.0;
    public static final double  kCoralArmMass                   = Units.lbsToKilograms(15); // Kilograms
    public static final double  kCoralArmLength                 = Inches.of(31).in(Meters);
    public static final Angle   kCoralArmStartingAngle          = Degrees.of(-85);
    public static final Angle   kCoralArmMinAngle               = Degrees.of(-92);
    public static final Angle   kCoralArmMaxAngle               = Degrees.of(87);
    public static final double  kCoralArmRampRate               = 0.5;
    public static final Angle   kCoralArmOffsetToHorizantalZero = Degrees.of(268.6);
    public static final boolean kCoralArmInverted               = true;
    public static final double  kCoralArmMaxVelocityRPM
                                                                = CoralArm.convertCoralAngleToSensorUnits(Degrees.of(
        10)).per(
        Second).in(RPM);
    public static final double  kCoralArmMaxAccelerationRPMperSecond
                                                                = CoralArm.convertCoralAngleToSensorUnits(Degrees.of(
                                                                              5)).per(
                                                                              Second).per(Second)
                                                                          .in(RPM.per(Second));
    public static final int     kCoralArmStallCurrentLimitAmps  = 40;

  }


  public static class ElevatorConstants
  {

    public static final double   kElevatorKp              = 33.966;
    public static final double   kElevatorKi              = 0;
    public static final double   kElevatorKd              = 9.4456;
    public static final double   kElevatorkS              = 0.21471; // volts (V)
    public static final double   kElevatorkV              = 10.39;//10.773; // volt per velocity (V/(m/s))
    public static final double   kElevatorkA              = 0; // volt per acceleration (V/(m/s²))
    public static final double   kElevatorkG              = 0.23861; // volts (V)
    public static       double   kMaxVelocity             = Meters.of(1).per(Second).in(MetersPerSecond);
    public static       double   kMaxAcceleration         = Meters.of(0.5).per(Second).per(Second).in(
        MetersPerSecondPerSecond);
    public static final double   kElevatorGearing         = 12.0;
    public static final double   kElevatorSproketTeeth    = 22;
    public static final double   kElevatorPitch           = Units.inchesToMeters(0.25);
    public static final double   kElevatorDrumRadius      = (kElevatorSproketTeeth * kElevatorPitch) / (2 * Math.PI);
    // radius = Circumference / (2 pi)
    public static final double   kCarriageMass            = Units.lbsToKilograms(16); // kg
    // Encoder is reset to measure 0 at the bottom, so minimum height is 0.
    public static final double   kMinElevatorHeightMeters = Units.inchesToMeters(0);//min height / 10
    public static final double   kMaxElevatorHeightMeters = Units.inchesToMeters(30);
    // Encoder is reset to measure 0 at the bottom, so minimum height is 0.
    public static final Distance kLaserCANOffset          = Meters.of(0.27);

    public static final Distance kStartingHeightSim        = Meters.of(0);
    public static final Distance kMinElevatorHeight        = Meters.of(kMinElevatorHeightMeters);
    public static final Distance kMaxElevatorHeight        = Meters.of(kMaxElevatorHeightMeters);
    public static final double   kElevatorAllowableError   = RobotBase.isSimulation() ? Units.inchesToMeters(1)
                                                                                      : 0.07;
    public static       double   kElevatorRampRate         = 0.1;
    public static       int      kElevatorCurrentLimit     = 40;
    public static final double   kElevatorUnextendedHeight = Units.inchesToMeters(41.5);
  }

  public static class IntakeConstants
  {

    public static final double AlgaeIntakeSpeeds  = 0.5;
    public static final double AlgaeOuttakeSpeeds = -0.8;

    public static final int    k_wristCurrentLimit       = 40;
    public static final double k_wristClosedLoopRampRate = 0.25;
    public static final double AlgaeHoldSpeed = 0.2;

  }

  public static class ClimberConstants
  {

    public static final double kClimbSpeed = 0.8;

  }
}