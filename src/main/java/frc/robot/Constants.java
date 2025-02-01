// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Second;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.RobotMath;
import frc.robot.RobotMath.AlgaeArm;
import frc.robot.RobotMath.CoralArm;
import frc.robot.RobotMath.Elevator;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean constants. This
 * class should not be used for any other purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */

public final class Constants {
  public static final Mechanism2d         sideRobotView = new Mechanism2d(AlgaeArmConstants.kAlgaeArmLength * 2,
                                                                          ElevatorConstants.kMaxElevatorHeight.in(
                                                                              Meters) +
                                                                          AlgaeArmConstants.kAlgaeArmLength);
  public static final MechanismRoot2d     kElevatorCarriage;
  public static final MechanismLigament2d kAlgaeArmMech;
  public static final MechanismLigament2d kCoralArmMech;
  public static final MechanismLigament2d kElevatorTower;

  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kOperatorControllerPort = 1;
    public static final double DEADBAND = 0.05;
  }

  public static final double maxSpeed = 7;

  public static final int kMotorPort       = 0;
  public static final int kEncoderAChannel = 0;
  public static final int kEncoderBChannel = 1;
  public static final int kJoystickPort    = 0;

  public static class targetingConstants
  {

    public static final double positiveScootch = Units.inchesToMeters(5);
    public static final double negitiveScootch = Units.inchesToMeters(-5);
    public static final double scootchBack     = Units.inchesToMeters(12);
  }

  public static class WristConstants
  {

    public static final double kWristMomentOfInertia = 0.00032; // kg * m^2
    public static final double kWristGearRatio       = 1.0;

    public static class RollerConstants
    {

      public static final double kWristMomentOfInertia = 0.00032; // kg * m^2
      public static final double kWristGearRatio       = 1.0;
    }
  }


  static
  {
    kElevatorCarriage = Constants.sideRobotView.getRoot("ElevatorCarriage",
                                                        AlgaeArmConstants.kAlgaeArmLength,
                                                        ElevatorConstants.kStartingHeightSim.in(
                                                    Meters));//The pivot
    kAlgaeArmMech = kElevatorCarriage.append(
        new MechanismLigament2d(
            "AlgaeArm",
            AlgaeArmConstants.kAlgaeArmLength,
            AlgaeArmConstants.kAlgaeArmStartingAngle.in(Degrees),
            6,
            new Color8Bit(Color.kYellow)));
    kCoralArmMech = kElevatorCarriage.append(
        new MechanismLigament2d(
            "CoraleArm",
            CoralArmConstants.kCoralArmLength,
            CoralArmConstants.kCoralArmStartingAngle.in(Degrees),
            6,
            new Color8Bit(Color.kOrange)));

    kElevatorTower = kElevatorCarriage.append(new MechanismLigament2d(
        "Elevator",
        ElevatorConstants.kStartingHeightSim.in(Meters),
        -90,
        6,
        new Color8Bit(Color.kRed)));
  }

  public static class AlgaeArmConstants{
  public static int algaeArmMotorID = 15;
  
  public static final String kAlgaeArmPositionKey = "ArmPosition";
  public static final String kAlgaeArmPKey = "ArmP";
  
  // The P gain for the PID controller that drives this arm.
    public static final double kAlgaeArmKp                     = 2.0691;
    public static final double kAlgaeArmKi                     = 0;
    public static final double kAlgaeArmKd                     = 0.0;
    public static final Angle  kAlgaeArmAllowedClosedLoopError = AlgaeArm.convertAlgaeAngleToSensorUnits(Degrees.of(0.01));

    public static final double  kAlgaeArmReduction                   = 200;
    public static final double  kAlgaeArmMass                        = 8.0; // Kilograms
    public static final double  kAlgaeArmLength                      = Inches.of(72).in(Meters);
    public static final Angle   kAlgaeArmStartingAngle               = Degrees.of(0);
    public static final Angle   kAlgaeArmMinAngle                    = Degrees.of(-75);
    public static final Angle   kAlgaeArmMaxAngle                    = Degrees.of(255);
    public static final double  kAlgaeArmRampRate                    = 0.5;
    public static final Angle   kAlgaeArmOffsetToHorizantalZero      = Rotations.of(0);
    public static final boolean kAlgaeArmInverted                    = false;
    public static final double  kAlgaeArmMaxVelocityRPM              = AlgaeArm.convertAlgaeAngleToSensorUnits(Degrees.of(90)).per(
        Second).in(RPM);
    public static final double  kAlgaeArmMaxAccelerationRPMperSecond = AlgaeArm.convertAlgaeAngleToSensorUnits(Degrees.of(180)).per(
                                                                         Second).per(Second)
                                                                     .in(RPM.per(Second));
    public static final int     kAlgaeArmStallCurrentLimitAmps       = 40;

    public static final double kAlgaeArmkS = 0; // volts (V)
    public static final double kAlgaeArmkG = 0; // volts (V)
    public static final double kAlgaeArmKv = 0; // volts per velocity (V/RPM)
    public static final double kAlgaeArmKa = 0; // volts per acceleration (V/(RPM/s))
  
  
  public static final double kAlgaeAngleAllowableError = 1;//degree, for testing whether it's aroundAngle
  
  }
  public static class CoralArmConstants{
    public static int coralArmMotorID = 14;
    
    public static final String kCoralArmPositionKey = "ArmPosition";
    public static final String kCoralArmPKey = "ArmP";
    
    // The P gain for the PID controller that drives this arm.
      public static final double kCoralArmKp                     = 2.0691;
      public static final double kCoralArmKi                     = 0;
      public static final double kCoralArmKd                     = 0.0;
      public static final Angle  kCoralArmAllowedClosedLoopError = CoralArm.convertCoralAngleToSensorUnits(Degrees.of(0.01));
  
      public static final double  kCoralArmReduction                   = 200;
      public static final double  kCoralArmMass                        = 8.0; // Kilograms
      public static final double  kCoralArmLength                      = Inches.of(72).in(Meters);
      public static final Angle   kCoralArmStartingAngle               = Degrees.of(0);
      public static final Angle   kCoralArmMinAngle                    = Degrees.of(-75);
      public static final Angle   kCoralArmMaxAngle                    = Degrees.of(255);
      public static final double  kCoralArmRampRate                    = 0.5;
      public static final Angle   kCoralArmOffsetToHorizantalZero      = Rotations.of(0);
      public static final boolean kCoralArmInverted                    = false;
      public static final double  kCoralArmMaxVelocityRPM              = CoralArm.convertCoralAngleToSensorUnits(Degrees.of(90)).per(
          Second).in(RPM);
      public static final double  kCoralArmMaxAccelerationRPMperSecond = CoralArm.convertCoralAngleToSensorUnits(Degrees.of(180)).per(
                                                                           Second).per(Second)
                                                                       .in(RPM.per(Second));
      public static final int     kCoralArmStallCurrentLimitAmps       = 40;
  
      public static final double kCoralArmkS = 0; // volts (V)
      public static final double kCoralArmkG = 0; // volts (V)
      public static final double kCoralArmKv = 0; // volts per velocity (V/RPM)
      public static final double kCoralArmKa = 0; // volts per acceleration (V/(RPM/s))
    
    
    public static final double kCoralAngleAllowableError = 1;//degree, for testing whether it's aroundAngle
    
    }



  public static class ElevatorConstants
  {

    public static int elevatorMotorID = 13;

    public static final double kElevatorKp = 5;
    public static final double kElevatorKi = 0;
    public static final double kElevatorKd = 0;

    public static final double kElevatorkS = 0.0; // volts (V)
    public static final double kElevatorkG = 0.762; // volts (V)
    public static final double kElevatorkV = 0.762; // volt per velocity (V/(m/s))
    public static final double kElevatorkA = 0.0; // volt per acceleration (V/(m/s²))

    public static final double kElevatorGearing    = 10.0;
    public static final double kElevatorDrumRadius = Units.inchesToMeters(2.0);
    public static final double kCarriageMass       = 4.0; // kg


    // Encoder is reset to measure 0 at the bottom, so minimum height is 0.
    public static final double kMinElevatorHeightMeters = 0;//min height / 10
    public static final double kMaxElevatorHeightMeters = 10.25;

    //public static final double kElevatorMaxVelocity = 3.5;
    //public static final double kElevatorMaxAcceleration = 2.5;


   
    // Encoder is reset to measure 0 at the bottom, so minimum height is 0.
    public static final Distance kLaserCANOffset    = Inches.of(3);
    public static final Distance kStartingHeightSim = Meters.of(0);
    public static final Distance kMinElevatorHeight = Meters.of(0.0);
    public static final Distance kMaxElevatorHeight = Meters.of(10.25);

    public static double kElevatorRampRate = 1;
    public static int    kElevatorCurrentLimit = 40;
    public static double kMaxVelocity = Elevator.convertDistanceToRotations(Meters.of(1)).per(Second).in(RPM);
    public static double kMaxAcceleration = Elevator.convertDistanceToRotations(Meters.of(2)).per(Second).per(Second)
                                                    .in(RPM.per(Second));
  
    public static final double kElevatorAllowableError = 1;
    public static final double kLowerToScoreHeight     = Units.inchesToMeters(6);
  }

  public static class IntakeConstants
  {

    public static final double AlgaeIntakeSpeeds  = 0.8;
    public static final double AlgaeOuttakeSpeeds = -0.8;

    public static final int    coralWristMotorID   = 16;
    public static final int    coralRollerMotorID  = 17;
    public static final double defaultrRollerSpeed = 0;
    public static final double kIntakeReduction    = 0;
    public static       double kMinAngleRads;
    public static       double kMaxAngleRads;
    public static       double kIntakeLength;
  }

}