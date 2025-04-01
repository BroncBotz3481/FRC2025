//
// Source code recreated from a .class file by IntelliJ IDEA
// (powered by FernFlower decompiler)
//

package utils;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.Trajectory.State;
import edu.wpi.first.math.util.Units;

public class ProfiledHolonomicDriveController
{

  private Pose2d     m_poseError     = Pose2d.kZero;
  private Rotation2d m_rotationError = Rotation2d.kZero;
  private Pose2d     m_poseTolerance = Pose2d.kZero;
  private boolean    m_enabled       = true;

  private final ProfiledPIDController m_xController;
  private final ProfiledPIDController m_yController;
  private final ProfiledPIDController m_thetaController;

  private boolean m_firstRun = true;



  /**
   * Constructs a holonomic drive controller
   *
   * @param xController     A profiled PID Controller to respond to error in the field-relative x direction. Translation
   *                        is in Meters.
   * @param yController     A profiled PID controller to respond to error int he field-relative y direction. Translation
   *                        is in Meters.
   * @param thetaController A profiled PID controller to respond to error in angle. Angle is in radians.
   */
  public ProfiledHolonomicDriveController(ProfiledPIDController xController, ProfiledPIDController yController,
                                          ProfiledPIDController thetaController)
  {
    this.m_xController = xController;
    this.m_yController = yController;
    this.m_thetaController = thetaController;
    this.m_thetaController.enableContinuousInput((double) 0.0F, Units.degreesToRadians((double) 360.0F));
  }

  /**
   * Constructs a holonomic drive controller
   *
   * @param translationController A profiled PID Controller to respond to error in the field-relative x and y direction.
   *                              Translation is in Meters.
   * @param thetaController       A profiled PID controller to respond to error in angle. Angle is in radians.
   */
  public ProfiledHolonomicDriveController(ProfiledPIDController translationController,
                                          ProfiledPIDController thetaController)
  {
    this.m_xController = translationController;
    this.m_yController = new ProfiledPIDController(translationController.getP(),
                                                   translationController.getI(),
                                                   translationController.getD(),
                                                   translationController.getConstraints());
    this.m_thetaController = thetaController;

    this.m_yController.setTolerance(this.m_xController.getPositionTolerance(),
                                    this.m_xController.getVelocityTolerance());
    this.m_thetaController.enableContinuousInput((double) 0.0F, Units.degreesToRadians((double) 360.0F));
  }

  /**
   * Returns true if the pose error is within tolerance of the reference
   *
   * @return True if the pose error is within tolerance of the reference.
   */
  public boolean atReference()
  {
    Translation2d eTranslate   = this.m_poseError.getTranslation();
    Rotation2d    eRotate      = this.m_rotationError;
    Translation2d tolTranslate = this.m_poseTolerance.getTranslation();
    Rotation2d    tolRotate    = this.m_poseTolerance.getRotation();
    // System.out.println("x: "+Units.metersToInches(eTranslate.getX())+" y: "+Units.metersToInches(eTranslate.getY())+" rot: "+eRotate.getDegrees());
    return Math.abs(eTranslate.getX()) < tolTranslate.getX() && Math.abs(eTranslate.getY()) < tolTranslate.getY() &&
           Math.abs(eRotate.getRadians()) < tolRotate.getRadians();
  }

  /**
   * Set the pose error which is considered tolerance for use with
   * {@link ProfiledHolonomicDriveController#atReference()}
   *
   * @param tolerance The {@link Pose2d} error which is tolerable.
   */
  public void setTolerance(Pose2d tolerance)
  {
    this.m_poseTolerance = tolerance;
  }

  /**
   * Returns the next output of the holonomic drive controller
   *
   * @param currentPose                          The current pose, as measured by odometry.
   * @param trajectoryPose                       The desired trajectory pose, as sampled for the current timestamp.
   * @param desiredLinearVelocityMetersPerSecond The desired linear velocity
   * @param desiredHeading                       The desired heading.
   * @return The next output of the holonomic drive controller. The output is robot-relative.
   */
  public ChassisSpeeds calculate(Pose2d currentPose, Pose2d trajectoryPose, double desiredLinearVelocityMetersPerSecond,
                                 Rotation2d desiredHeading)
  {
    if (this.m_firstRun)
    {
      this.m_thetaController.reset(currentPose.getRotation().getRadians());
      this.m_firstRun = false;
    }

    double xFF = desiredLinearVelocityMetersPerSecond * trajectoryPose.getRotation().getCos();
    double yFF = desiredLinearVelocityMetersPerSecond * trajectoryPose.getRotation().getSin();
    double thetaFF = this.m_thetaController.calculate(currentPose.getRotation().getRadians(),
                                                      desiredHeading.getRadians());
    this.m_poseError = trajectoryPose.relativeTo(currentPose);
    this.m_rotationError = desiredHeading.minus(currentPose.getRotation());
    if (!this.m_enabled)
    {
      return ChassisSpeeds.fromFieldRelativeSpeeds(xFF, yFF, thetaFF, currentPose.getRotation());
    } else
    {
      double xFeedback = this.m_xController.calculate(currentPose.getX(), trajectoryPose.getX());
      double yFeedback = this.m_yController.calculate(currentPose.getY(), trajectoryPose.getY());
      return ChassisSpeeds.fromFieldRelativeSpeeds(xFF + xFeedback,
                                                   yFF + yFeedback,
                                                   thetaFF,
                                                   currentPose.getRotation());
    }
  }

  /**
   * Returns the next output of the holonomic drive controller
   *
   * @param currentPose    The current pose as measured by odometry or pose estimator.
   * @param desiredState   The desired trajectory pose, as sampled for the current timestamp.
   * @param desiredHeading The desired heading.
   * @return The next output of the holonomic drive controller.
   */
  public ChassisSpeeds calculate(Pose2d currentPose, Trajectory.State desiredState, Rotation2d desiredHeading)
  {
    return this.calculate(currentPose, desiredState.poseMeters, desiredState.velocityMetersPerSecond, desiredHeading);
  }

  /**
   * Reset the controller starting pose to current pose.
   *
   * @param currentPose The current pose as measured by odometry or pose estimator.
   */
  public void reset(Pose2d currentPose)
  {
    this.m_xController.reset(currentPose.getTranslation().getX());
    this.m_yController.reset(currentPose.getTranslation().getY());
    this.m_thetaController.reset(currentPose.getRotation().getRadians());
  }

  /**
   * Reset the controller starting pose to current pose.
   *
   * @param currentPose     The current pose as measured by odometry or pose estimator.
   * @param currentVelocity The current field-relative velocity as measured by odometry or pose estimator.
   */
  public void reset(Pose2d currentPose, ChassisSpeeds currentVelocity)
  {
    this.m_xController.reset(currentPose.getTranslation().getX(), currentVelocity.vxMetersPerSecond);
    this.m_yController.reset(currentPose.getTranslation().getY(), currentVelocity.vyMetersPerSecond);
    this.m_thetaController.reset(currentPose.getRotation().getRadians(), currentVelocity.omegaRadiansPerSecond);
  }

  /**
   * Enables and disables the controller for troubleshooting problems. When
   * {@link ProfiledHolonomicDriveController#calculate(Pose2d, State, Rotation2d)} is called on a disabled controller,
   * only feedforward values are returned
   *
   * @param enabled If the controller is enabled or not.
   */
  public void setEnabled(boolean enabled)
  {
    this.m_enabled = enabled;
  }

  /**
   * Returns the X controller
   *
   * @return X ProfiledPIDController
   */
  public ProfiledPIDController getXController()
  {
    return this.m_xController;
  }

  /**
   * Returns the Y controller
   *
   * @return Y ProfiledPIDController
   */
  public ProfiledPIDController getYController()
  {
    return this.m_yController;
  }

  /**
   * Returns the heading controller.
   *
   * @return Heading ProfiledPIDController.
   */
  public ProfiledPIDController getThetaController()
  {
    return this.m_thetaController;
  }
}
