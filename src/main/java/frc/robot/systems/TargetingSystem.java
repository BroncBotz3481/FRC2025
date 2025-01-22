package frc.robot.systems;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;

public class TargetingSystem {

    private AprilTagFieldLayout fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape);

    public double getTargetBranchHeightMeters() {
        switch (targetBranchLevel) {

            case L2 -> {
                return Units.inchesToMeters(32);
            }
            case L3 -> {
                return Units.inchesToMeters(48);
            }
            case L4 -> {
                return Units.inchesToMeters(72);
            }
        }
        return 0;
    }

    public double getTargetBranchAlgaeArmAngle() {
        return 0;
    }

    public double getTargetBranchCoralArmAngle() {
        switch (targetBranchLevel) {
            case L2 -> {
                return 30;
            }
            case L3 -> {
                return 30;
            }
            case L4 -> {
                return 60;
            }
        }
        return 0;
    }


    private ReefBranch targetBranch;
    private ReefBranchLevel targetBranchLevel;

    public void setTarget(ReefBranch targetBranch, ReefBranchLevel targetBranchLevel) {
        this.targetBranch = targetBranch;
        this.targetBranchLevel = targetBranchLevel;
    }

    public void left() {
        if (targetBranch == ReefBranch.H) {
            targetBranch = ReefBranch.I;
        }
    }

    public Pose2d getTargetPose() {
        if (DriverStation.getAlliance().isPresent()) {
            if (DriverStation.getAlliance().get() == DriverStation.Alliance.Red) {
                switch (targetBranch) {
                    case A -> {
                        fieldLayout.getTagPose(6)
                                .get().toPose2d().plus(new Transform2d(Units.inchesToMeters(5), 0.01, Rotation2d.fromDegrees(30)));
                    }
                    case B -> {
                        return new Pose2d(1,1,Rotation2d.fromDegrees(30));
                    }
                    case C -> {
                    }
                    case D -> {
                    }
                    case E -> {
                    }
                    case F -> {
                    }
                    case G -> {
                    }
                    case H -> {
                    }
                    case I -> {
                    }
                    case J -> {
                    }
                    case K -> {
                    }
                    case L -> {
                    }
                }
            } else {
                switch (targetBranch) {
                    case A -> {
                    }
                    case B -> {
                    }
                    case C -> {
                    }
                    case D -> {
                    }
                    case E -> {
                    }
                    case F -> {
                    }
                    case G -> {
                    }
                    case H -> {
                    }
                    case I -> {
                    }
                    case J -> {
                    }
                    case K -> {
                    }
                    case L -> {
                    }
                }
            }
        }
        return Pose2d.kZero;
    }

    public enum ReefBranch {
        A,
        B,
        C,
        D,
        E,
        F,
        G,
        H,
        I,
        J,
        K,
        L
    }

    ;

    public enum ReefBranchLevel {
        L2,
        L3,
        L4
    }

}
