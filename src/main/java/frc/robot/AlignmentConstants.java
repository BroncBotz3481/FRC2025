package frc.robot;

import com.pathplanner.lib.config.PIDConstants;

public class AlignmentConstants
{
  public static class DriveToPose
  {

    public static final boolean enableDriveFeedFords = true;

    public static final PIDConstants translationPID = new PIDConstants(5.0,0,0);
    public static final double maximumVelocityMetersPerSecond = 0.55;
    public static final double maximumAccelerationMetersPerSecondSquared = 0.20;

    public static final PIDConstants rotationPID = new PIDConstants(5.0,0,0);
    public static final double maximumAngularVelocityDegreesPerSecond = 90;
    public static final double maximumAngularAccelerationDegreesPerSecondSquared = 45;

  }
}
