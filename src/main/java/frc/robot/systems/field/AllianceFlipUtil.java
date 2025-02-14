// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.systems.field;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;

public class AllianceFlipUtil
{

  public static double applyX(double x)
  {
    return shouldFlip() ? FieldConstants.fieldLength - x : x;
  }

  public static double applyY(double y)
  {
    return shouldFlip() ? FieldConstants.fieldWidth - y : y;
  }

  public static Translation2d apply(Translation2d translation)
  {
    return new Translation2d(applyX(translation.getX()), applyY(translation.getY()));
  }

  public static Rotation2d apply(Rotation2d rotation)
  {
    return shouldFlip() ? rotation.rotateBy(Rotation2d.kPi) : rotation;
  }

  public static Pose2d apply(Pose2d pose)
  {
    return shouldFlip()
           ? new Pose2d(apply(pose.getTranslation()), apply(pose.getRotation()))
           : pose;
  }

  public static Pose2d flip(Pose2d pose)
  {
    return new Pose2d(apply(pose.getTranslation()), apply(pose.getRotation()));
  }

  public static boolean shouldFlip()
  {
    return (DriverStation.getAlliance().isPresent()
            && DriverStation.getAlliance().get() == DriverStation.Alliance.Red);
  }
}
