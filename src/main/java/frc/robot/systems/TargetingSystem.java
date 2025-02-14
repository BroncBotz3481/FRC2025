package frc.robot.systems;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.systems.field.AllianceFlipUtil;
import frc.robot.systems.field.FieldConstants.Reef;
import frc.robot.systems.field.FieldConstants.ReefHeight;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.function.Supplier;
import java.util.stream.Collectors;

//targetting system should be able to select either left or right side of the branch
//then select what level we want
// that go to the nearest side of the reef and load.


public class TargetingSystem
{

  private AprilTagFieldLayout fieldLayout              = AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape);
  private ReefBranch          targetBranch;
  private ReefBranchLevel     targetBranchLevel;
  private Transform2d         robotBranchScoringOffset = new Transform2d(Inches.of(24).in(Meters),
                                                                         Inches.of(0).in(Meters),
                                                                         Rotation2d.fromDegrees(0));

  private List<Pose2d>            reefBranches                 = null;
  private List<Pose2d>            allianceRelativeReefBranches = null;
  private Map<Pose2d, ReefBranch> reefPoseToBranchMap          = null;

  public TargetingSystem()
  {
    reefBranches = new ArrayList<>();
    reefPoseToBranchMap = new HashMap<>();
    for (int branchPositionIndex = 0; branchPositionIndex < Reef.branchPositions.size(); branchPositionIndex++)
    {
      Map<ReefHeight, Pose3d> branchPosition = Reef.branchPositions.get(branchPositionIndex);
      Pose2d                  targetPose     = AllianceFlipUtil.apply(branchPosition.get(ReefHeight.L4).toPose2d());
      reefBranches.add(targetPose);
      reefPoseToBranchMap.put(targetPose, ReefBranch.values()[branchPositionIndex]);
      reefPoseToBranchMap.put(AllianceFlipUtil.flip(targetPose), ReefBranch.values()[branchPositionIndex]);
    }
  }


  public double getTargetBranchHeightMeters()
  {
    switch (targetBranchLevel)
    {
      case L2 ->
      {
        return ReefHeight.L2.height;
      }
      case L3 ->
      {
        return ReefHeight.L3.height;
      }
      case L4 ->
      {
        return ReefHeight.L4.height;
      }
    }
    return 0;
  }

  public double getTargetBranchAlgaeArmAngle()
  {

    return 0;
  }

  public double getTargetBranchCoralArmAngle()
  {
    switch (targetBranchLevel)
    {
      case L2 ->
      {
        return ReefHeight.L2.pitch;
      }
      case L3 ->
      {
        return ReefHeight.L3.pitch;
      }
      case L4 ->
      {
        return ReefHeight.L4.pitch;
      }
    }
    return 0;
  }

  public void setTarget(ReefBranch targetBranch, ReefBranchLevel targetBranchLevel)
  {
    this.targetBranch = targetBranch;
    this.targetBranchLevel = targetBranchLevel;
  }

  public Command setTargetCommand(ReefBranch targetBranch, ReefBranchLevel targetBranchLevel)
  {
    return Commands.runOnce(() -> setTarget(targetBranch, targetBranchLevel));
  }

  public Command setBranchCommand(ReefBranch branch)
  {
    return Commands.runOnce(() -> {
      targetBranch = branch;
    });
  }

  public Command setBranchLevel(ReefBranchLevel level)
  {
    return Commands.runOnce(() -> {
      targetBranchLevel = level;
    });
  }

  public void left()
  {
    if (targetBranch == ReefBranch.H)
    {
      targetBranch = ReefBranch.I;
    }
  }

  public Pose2d getTargetPose()
  {
    Pose2d scoringPose = Pose2d.kZero;
    if (targetBranch != null)
    {
      scoringPose = Reef.branchPositions.get(targetBranch.ordinal()).get(ReefHeight.L2).toPose2d()
                                        .plus(robotBranchScoringOffset);
    }
    return AllianceFlipUtil.apply(scoringPose);
  }


  public Pose2d autoTarget(Supplier<Pose2d> currentPose)
  {
    if (allianceRelativeReefBranches == null)
    {
      allianceRelativeReefBranches = reefBranches.stream()
                                                 .map(AllianceFlipUtil::apply)
                                                 .collect(Collectors.toList());
    }
    Pose2d selectedTargetPose = currentPose.get().nearest(allianceRelativeReefBranches);
    targetBranch = reefPoseToBranchMap.get(selectedTargetPose);
    return selectedTargetPose;
  }

  public Command autoTargetCommand(Supplier<Pose2d> currentPose)
  {
    return Commands.runOnce(() -> {
      autoTarget(currentPose);
    });
  }

  public enum ReefBranch
  {
    A,
    B,
    K,
    L,
    I,
    J,
    G,
    H,
    E,
    F,
    C,
    D
  }


  public enum ReefBranchLevel
  {
    L2,
    L3,
    L4
  }

}
