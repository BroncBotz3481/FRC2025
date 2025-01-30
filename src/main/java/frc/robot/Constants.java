// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kOperatorControllerPort = 1;
    public static final double DEADBAND = 0.05;
  }
  public static final double maxSpeed = 7;

  public static final int kMotorPort = 0;
  public static final int kEncoderAChannel = 0;
  public static final int kEncoderBChannel = 1;
  public static final int kJoystickPort = 0;

  public static class targetingConstants{
    public static final double positiveScootch = Units.inchesToMeters(5);
    public static final double negitiveScootch = Units.inchesToMeters(-5);
    public static final double scootchBack = Units.inchesToMeters(12);
  }
  

  public static class ArmConstants{
  public static int coralArmMotorID = 14;
  public static int algaeArmMotorID = 15;
  
  public static final String kArmPositionKey = "ArmPosition";
  public static final String kArmPKey = "ArmP";

  // The P gain for the PID controller that drives this arm.
  public static final double kDefaultArmKp = 80.0;
  public static final double kArmKi = 0.0;
  public static final double kArmKd = 0.5;

  // distance per pulse = (angle per revolution) / (pulses per revolution)
  //  = (2 * PI rads) / (4096 pulses)
  //public static final double kArmEncoderDistPerPulse = 2.0 * Math.PI / 4096;

  public static final double kArmReduction = 200;
  public static final double kArmMass = 8.0; // Kilograms 
  public static final double kArmLength = Units.inchesToMeters(30);
  public static final double kMinAngleRads = Units.degreesToRadians(-75);
  public static final double kMaxAngleRads = Units.degreesToRadians(255);
  
  public static final double kArmkS = 0.0; // volts (V)
  public static final double kArmkG = 0.762; // volts (V)
  public static final double kArmkV = 0.762; // volt per velocity (V/(m/s))
  public static final double kArmkA = 0.0; // volt per acceleration (V/(m/s²))

  public static final double kAngleAllowableError = 1;//degree, for testing whether it's aroundAngle
  
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

    public static final double kElevatorGearing = 10.0;
    public static final double kElevatorDrumRadius = Units.inchesToMeters(2.0);
    public static final double kCarriageMass = 4.0; // kg


   // Encoder is reset to measure 0 at the bottom, so minimum height is 0.
    public static final double kMinElevatorHeightMeters = 0;//min height / 10
    public static final double kMaxElevatorHeightMeters = 10.25;

    //public static final double kElevatorMaxVelocity = 3.5;
    //public static final double kElevatorMaxAcceleration = 2.5;

    public static final double kElevatorAllowableError = 1;
    public static final double kLowerToScoreHeight = Units.inchesToMeters(6);
  }
  public static class IntakeConstants{
    public static final double AlgaeIntakeSpeeds = 0.8;
    public static final double AlgaeOuttakeSpeeds = -0.8;
  }

}
