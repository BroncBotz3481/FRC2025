package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import com.pathplanner.lib.config.PIDConstants;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import utils.ProfiledHolonomicDriveController;

public class AlignmentConstants
{

  public static class DriveToPose
  {

    public static final Distance translationTolerance = Inches.of(0.25);
    public static final Angle    rotationTolerance    = Degrees.of(0.5);
    public static final Pose2d   poseTolerance        = new Pose2d(Inches.of(0.3),
                                                                    Inches.of(0.3),
                                                                   new Rotation2d(rotationTolerance));

    public static final Distance MAX_AUTO_DRIVE_REEF_DISTANCE  = Meters.of(2);
    public static final Distance MAX_AUTO_DRIVE_ALGAE_DISTANCE = Meters.of(2);

    public static final boolean enableDriveFeedFords = true;

    public static final PIDConstants          translationPID                            = new PIDConstants(10,
                                                                                                           0,
                                                                                                           0); // 4.0, 0,0
    public static final double                maximumVelocityMetersPerSecond            = 1.3;
    public static final double                maximumAccelerationMetersPerSecondSquared = 0.95;
    public static final PIDController         translationController                     = new PIDController(
        translationPID.kP,
        translationPID.kI,
        translationPID.kD);
    public static final ProfiledPIDController profiledTranslationController             = new ProfiledPIDController(
        translationPID.kP,
        translationPID.kI,
        translationPID.kD,
        new Constraints(maximumVelocityMetersPerSecond, maximumAccelerationMetersPerSecondSquared));

    public static final PIDConstants          rotationPID                                       = new PIDConstants(8.0,
                                                                                                                   // 3.0
                                                                                                                   0,
                                                                                                                   0);
    public static final double                maximumAngularVelocityDegreesPerSecond            = 90; // 360
    public static final double                maximumAngularAccelerationDegreesPerSecondSquared = Math.pow(
        maximumAngularVelocityDegreesPerSecond,
        2);
    public static final ProfiledPIDController rotationController                                = new ProfiledPIDController(
        rotationPID.kP,
        rotationPID.kI,
        rotationPID.kD,
        new TrapezoidProfile.Constraints(maximumAngularVelocityDegreesPerSecond,
                                         maximumAngularAccelerationDegreesPerSecondSquared));

    public static HolonomicDriveController         driveController;
    public static ProfiledHolonomicDriveController profiledDriveController;

    static
    {
      translationController.setTolerance(translationTolerance.in(Meters));
      profiledTranslationController.setTolerance(translationTolerance.in(Meters));
      rotationController.setTolerance(rotationTolerance.in(Degrees));

      rotationController.enableContinuousInput(0, 360);
      // ^ Doesn't really do anything since it is overwritten inside ProfiledHolonomicDriveController.

      driveController = new HolonomicDriveController(
          translationController,
          translationController, // Might need to be copied to a new controller.
          rotationController);
      profiledDriveController = new ProfiledHolonomicDriveController(profiledTranslationController, rotationController);

      driveController.setTolerance(poseTolerance);
      profiledDriveController.setTolerance(poseTolerance);
    }


  }


}
