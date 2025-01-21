package frc.robot.systems;

public class TargetingSystem {

    public double getTargetBranchHeightMeters() {
        switch (targetBranchLevel)
        {
            case L1 -> {
                return 1.0;

            }
            case L2 -> {
                return 0;
            }
            case L3 -> {
                return 0;
            }
            case L4 -> {
                return 0;
            }
        }
        return 0;
    }
    public double getTargetBranchAlgaeArmAngle() {return 0;}
    public double getTargetBranchCoralArmAngle() {return 0;}


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
        L1,
        L2,
        L3,
        L4
    }

}
