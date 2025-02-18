// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;

import static edu.wpi.first.units.Units.*;
import frc.robot.RobotMath.Arm;

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
  public static final Mechanism2d sideView = new Mechanism2d(ArmConstants.kArmLength * 2,
          ArmConstants.kArmLength + ElevatorConstants.kElevatorLength);
  public static final MechanismRoot2d elevatorCarriage;
  public static final MechanismLigament2d armMech;
  public static final MechanismLigament2d elevatorMech;

  static{
    elevatorCarriage = Constants.sideView.getRoot("Elevator Carriage",
                                                        ArmConstants.kArmLength,
                                                        ElevatorConstants.kElevatorStartingHeightSim.in(Meters));
    armMech = elevatorCarriage.append(new MechanismLigament2d("Arm",
                                                        ArmConstants.kArmLength,
                                                        ArmConstants.kArmStartingAngle.in(Degrees),
                                                        6,
                                                        new Color8Bit(Color.kOrange)));
    elevatorMech = elevatorCarriage.append(new MechanismLigament2d("Elevator",
                                                        ElevatorConstants.kElevatorLength,
                                                        ElevatorConstants.kElevatorStartingAngle.in(Degrees),
                                                        6,
                                                        new Color8Bit(Color.kRed)));
  }



  public static final double              maxSpeed      = 7;
  public static class OperatorConstants
  {

    public static final int    kDriverControllerPort   = 0;
    public static final int    kOperatorControllerPort = 1;
    public static final double DEADBAND                = 0.05;
  }

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

  public static class ArmConstants{
    public static final int armMotorID = 15;
    public static final double kArmkS = 0;    // Volts (V)
    public static final double kArmkG = 1.53; // Volts (V)
    public static final double kArmkV = 1.58; // Volts Per Velocity (V/(rad/s))
    public static final double kArmkA = 0.08; // Volts Per Acceleration (V/(rad/s^2))
    public static final double kArmkP = 0.5;//?
    public static final double kArmkI = 0.0;
    public static final double kArmkD = 0.0;
    public static final double kArmReduction = 81;
    public static final double kArmMaxVelocity = Arm.convertArmAngleToSensorUnits(Degrees.of(90))//?
                                                .per(Second).in(RPM) ;
    public static final double kArmMaxAcceleration = Arm.convertArmAngleToSensorUnits(Degrees.of(180))
                                                    .per(Second).per(Second).in(RPM.per(Second)) ;
    public static final int ArmStallCurrentLimit = 40;
    public static final double ArmRampRate = 0.5;
    public static final boolean ArmInverted = false;
    public static final double kArmAllowedClosedLoopError = Arm.convertArmAngleToSensorUnits(Degrees.of(0.01)).in(Rotations);
    public static final double kArmLength = Inches.of(31).in(Meters);
    public static final double kArmMass = 8.0; // kg//?
    public static final Angle kArmMinAngle = Degrees.of(-90);
    public static final Angle kArmMaxAngle = Degrees.of(255);//?
    public static final Angle kArmStartingAngle = Degrees.of(0);
    public static final Angle kArmOffsetToHorizontalZero = Rotations.of(0);
    public static final double kArmDefaultTolerance = 1;

  }


  public static class ElevatorConstants {
    public static final double kElevatorKp = 5;//5
    public static final double kElevatorKi = 0;
    public static final double kElevatorKd = 0;//
    public static final double kMaxVelocity = Meters.of(4).per(Second).in(MetersPerSecond);
    public static final double kMaxAcceleration = Meters.of(8).per(Second).per(Second).in(MetersPerSecondPerSecond);
    public static final double kElevatorkS = 0.02;
    public static final double kElevatorkG = 0.9;
    public static final double kElevatorkV = 3.8;
    public static final double kElevatorkA = 0.17;
    public static final double kElevatorRampRate = 0.1;
    public static final double kElevatorGearing = 12.0;
    public static final double kElevatorCarriageMass = 4.0;
    public static final double kElevatorDrumRadius = Units.inchesToMeters(2.0);
    public static final double kElevatorMinHeightMeters = 0.0;
    public static final double kElevatorMaxHeightMeters = 10.25;
    public static final double kElevatorLength = Inches.of(33).in(Meters);
    public static final Distance kElevatorStartingHeightSim = Meters.of(0.0);
    public static final Angle kElevatorStartingAngle = Degrees.of(-90);
    public static final Distance kLaserCANOffset          = Inches.of(3);
    public static final double kElevatorDefaultTolerance = Inches.of(1).in(Meters);

    public static double kLowerToScoreHeight =  Units.inchesToMeters(6);;
  }


  public static class IntakeConstants
  {

    public static final double AlgaeIntakeSpeeds  = 0.8;
    public static final double AlgaeOuttakeSpeeds = -0.8;

    public static final int    coralWristMotorID   = 16;
    public static final int    coralRollerMotorID  = 17;
    public static       int    algaeRollerMotorID  = 18;
    public static final double defaultrRollerSpeed = 0;
    public static final double kIntakeReduction    = 0;

    public static final int    k_wristCurrentLimit       = 40;
    public static final double k_wristClosedLoopRampRate = 0.25;

    public static Angle  kMinAngle       = Degrees.of(-20);
    public static Angle  kMaxAngle       = Degrees.of(220);
    public static double kIntakeLength   = Inches.of(10).in(Meters);
    public static double kWristReduction = 1;
    public static double kIntakeMass     = 2.27; //kg
  }

}