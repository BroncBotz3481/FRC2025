package frc.robot.systems;

import edu.wpi.first.math.util.Units;

public class TargetingSystem {

    public double getTargetBranchHeightMeters() {
        switch (targetBranchLevel)
        {

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
    public double getTargetBranchAlgaeArmAngle() {return 0;}
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
        if(targetBranch == ReefBranch.H)
        {
            targetBranch = ReefBranch.I;
        }
    }

    enum ReefBranch {
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
    };
    enum ReefBranchLevel
    {
        L2,
        L3,
        L4
    }

}
