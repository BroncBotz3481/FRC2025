// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.units.Units;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.AlgaeArmSubsystem;
import frc.robot.subsystems.AlgaeIntakeSubsystem;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.CoralArmSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.FloorIntakeSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.systems.LoadingSystem;
import frc.robot.systems.ScoringSystem;
import frc.robot.systems.TargetingSystem;
import frc.robot.systems.TargetingSystem.ReefBranch;
import frc.robot.systems.TargetingSystem.ReefBranchLevel;
import frc.robot.systems.field.FieldConstants;
import frc.robot.systems.field.FieldConstants;
import frc.robot.systems.field.FieldConstants.Reef;
import frc.robot.systems.field.FieldConstants.ReefHeight;
import swervelib.SwerveInputStream;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meter;
import static edu.wpi.first.units.Units.Meters;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import java.util.Set;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
    // The robot's subsystems and commands are defined here...
    private final SwerveSubsystem drivebase = new SwerveSubsystem();
    // Replace with CommandPS4Controller or CommandJoystick if needed
    private final CommandXboxController m_driverController =
            new CommandXboxController(OperatorConstants.kDriverControllerPort);

    public static final CommandXboxController m_operatorController =
            new CommandXboxController(OperatorConstants.kOperatorControllerPort);

    private final ElevatorSubsystem elevator = new ElevatorSubsystem();
    private final CoralArmSubsystem coralArm = new CoralArmSubsystem();
    private final ClimberSubsystem climb = new ClimberSubsystem();
    private final AlgaeIntakeSubsystem algaeIntake = new AlgaeIntakeSubsystem();
    private final AlgaeArmSubsystem algaeArm = new AlgaeArmSubsystem();
    private final FloorIntakeSubsystem floorIntake = new FloorIntakeSubsystem();

    private final LoadingSystem loadingSystem = new LoadingSystem(coralArm, algaeArm, elevator);
    private final TargetingSystem targetingSystem = new TargetingSystem();
    private final ScoringSystem scoringSystem = new ScoringSystem(coralArm, elevator, drivebase, algaeIntake, algaeArm, loadingSystem, targetingSystem);

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */

    /** TO DO
    Change keyboard settings
    - add All pose in target system *get interns to do
    - Add all the buttons *get interns to do

    Fix Arm simulation, change angle of arm on start up, currently -90
    -difference between the goal and the actual location it reaches-because of consuming extra power?-pid tuning
    -it takes really long to move each arm????But all three sim is working

    set MAX MIN ANGLE for arm, ask cad team?-no limits (360 degrees 
    Change height of elevator.max height of the barge. currently in meters change that
    HEIGHT AND ANGLE FOR EACH LEVEL(ARM LENGTH?)-no idea yet

    (Bumpers: 6 inch
    Elevator alone Min: 39.25
    Unextended:41 1/2(from the ground)  Extended Elevator:71.094 so Second half of the elevator:29.594
    For the net: extended elev + algaeAngle(val?)
    
    -start from horizontal(degrees)
    -L4 +80 
`   Copy Math class
    Copy code from yagsl test code.

    Add sensors
   -for algae, using algaeInArm.get()?
   

   FIX QUESTION AND ASK LIMITS
    */

    public RobotContainer() {
        // Configure the trigger bindings
        DriverStation.silenceJoystickConnectionWarning(true);
        elevator.setDefaultCommand(elevator.setGoal(0));
        coralArm.setDefaultCommand(coralArm.setGoal(-90));
        climb.setDefaultCommand(climb.climbUp());
        algaeIntake.setDefaultCommand(algaeIntake.setAlgaeIntakeRoller(0));
        algaeArm.setDefaultCommand(algaeArm.setGoal(-90));
        floorIntake.setDefaultCommand(floorIntake.setCoralIntakeAngle(0));

        targetingSystem.setTarget(ReefBranch.G,  ReefBranchLevel.L2);
        drivebase.getSwerveDrive().field.getObject("REEF").setPose(targetingSystem.getTargetPose());
        configureBindings();
        //drivebase.setDefaultCommand(driveFieldOrientedAngularVelocity);
        drivebase.setDefaultCommand(!RobotBase.isSimulation() ? driveFieldOrientedAngularVelocity : driveFieldOrientedDirectAngleSim);
        NamedCommands.registerCommand("test", Commands.print("Hello World"));
    }

    // The real world (whats that?)
    SwerveInputStream driveAngularVelocity = SwerveInputStream.of(drivebase.getSwerveDrive(),
                    () -> m_driverController.getLeftY() * -1,
                    () -> m_driverController.getLeftX() * -1)
            .withControllerRotationAxis(m_driverController::getRightX)
            .deadband(OperatorConstants.DEADBAND)
            .scaleTranslation(0.8)
            .allianceRelativeControl(true);

    SwerveInputStream driveDirectAngle = driveAngularVelocity.copy().withControllerHeadingAxis(m_driverController::getRightX,
                    m_driverController::getRightY)
            .headingWhile(true);

    Command driveFieldOrientedDriectAngle = drivebase.driveFieldOriented(driveDirectAngle);

    Command driveFieldOrientedAngularVelocity = drivebase.driveFieldOriented(driveAngularVelocity);

    Command driveSetpointGen = drivebase.driveWithSetpointGeneratorFieldRelative(driveDirectAngle);
//Non reality code


    SwerveInputStream driveAngularVelocitySim = SwerveInputStream.of(drivebase.getSwerveDrive(),
                    () -> -m_driverController.getLeftY(),
                    () -> -m_driverController.getLeftX())
            .withControllerRotationAxis(() -> m_driverController.getRawAxis(2))
            .deadband(OperatorConstants.DEADBAND)
            .scaleTranslation(0.8)
            .allianceRelativeControl(true);
    // Derive the heading axis with math!
    SwerveInputStream driveDirectAngleSim = driveAngularVelocitySim.copy()
            .withControllerHeadingAxis(() -> Math.sin(
                            m_driverController.getRawAxis(
                                    2) * Math.PI) * (Math.PI * 2),
                    () -> Math.cos(
                            m_driverController.getRawAxis(
                                    2) * Math.PI) *
                            (Math.PI * 2))
            .headingWhile(true);

    Command driveFieldOrientedDirectAngleSim = drivebase.driveFieldOriented(driveDirectAngleSim);

    Command driveSetpointGenSim = drivebase.driveWithSetpointGeneratorFieldRelative(driveDirectAngleSim);


    /**
     * Use this method to define your trigger->command mappings. Triggers can be created via the
     * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
     * predicate, or via the named factories in {@link
     * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
     * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
     * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
     * joysticks}.
     */
    private void configureBindings() {
        // Schedule `ExampleCommand` when `exampleCondition` changes to `true`


        m_driverController.button(10).whileTrue(drivebase.sysIdDriveMotorCommand());
        m_driverController.button(9).whileTrue(drivebase.driveToPose(new Pose2d(new Translation2d
                (Meter.of(3),
                        Meter.of(4)),
                Rotation2d.fromDegrees(-180))));

        m_driverController.button(8).whileTrue(drivebase.driveToPose(new Pose2d(new Translation2d
                (Meter.of(5),
                        Meter.of(3)),
                Rotation2d.fromDegrees(0))));

        m_driverController.button(7).whileTrue(drivebase.driveToPose(new Pose2d(new Translation2d
                (Meter.of(3.5),
                        Meter.of(2.5)),
                Rotation2d.fromDegrees(125))));

        m_driverController.button(6).whileTrue(
                Commands.runOnce(() -> targetingSystem.setTarget(
                        TargetingSystem.ReefBranch.D,
                        TargetingSystem.ReefBranchLevel.L4))
                        .andThen(Commands.deferredProxy(()-> drivebase.driveToPose(targetingSystem.getTargetPose())))
                        .andThen(Commands.deferredProxy(scoringSystem::scoreCoral)));

        m_driverController.button(5).whileTrue(driveToSetPoint(6.1, 4, 125));
        m_driverController.button(4).whileTrue(driveToSetPoint(5.2, 5.2, -120));
        m_driverController.button(3).whileTrue(driveToSetPoint(3.3, 5.3, -50));
        //Processor
        m_driverController.button(2).whileTrue(driveToProcessor());
        //Human Playerstation                                                                                
        m_driverController.button(1).whileTrue(driveToHumanPlayer1());

        // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
        // cancelling on release.


        m_driverController.button(11).whileTrue(elevator.setGoal(3));
        m_driverController.button(12).whileTrue(elevator.setGoal(6));
        m_driverController.button(13).whileTrue(elevator.setGoal(9));
        m_driverController.button(14).whileTrue(coralArm.setGoal(45));
        m_driverController.button(15).whileTrue(coralArm.setGoal(90));
        m_driverController.button(16).whileTrue(setElevArm(9.5, 70));
        elevator.atHeight(5, 0.1).whileTrue(Commands.print("I AM ALIVE, YAAA HAAAAA"));

        m_driverController.button(19).whileTrue(algaeIntake.setAlgaeIntakeRoller(Constants.IntakeConstants.AlgaeOuttakeSpeeds));
        m_driverController.button(18).whileTrue(algaeIntake.setAlgaeIntakeRoller(Constants.IntakeConstants.AlgaeIntakeSpeeds));

        m_driverController.button(20).whileTrue(algaeArm.setGoal(45));
        m_driverController.button(21).whileTrue(algaeArm.setGoal(90));
        m_driverController.button(22).whileTrue(climb.climbDown());
        m_driverController.button(23).whileTrue(climb.climbUp());


        System.out.println(getMe()); // 0
        m_driverController.button(25).whileTrue(
                Commands.runOnce(() -> changeMe())
                        .andThen(Commands.print(""+getMe())) // 0
                        .andThen(() -> System.out.println(getMe()))); // 1

    }


    private void changeMe() {
        me = 1;
    }

    private double me = 0;

    private double getMe() {
        return me;
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // An example command will be run in autonomous
        return drivebase.getAutonomousCommand("New Auto");
    }

    public Command driveToSetPoint(double x, double y, double angle) {
        return drivebase.driveToPose(
                new Pose2d(new Translation2d
                        (Meter.of(x),
                                Meter.of(y)),
                        Rotation2d.fromDegrees(angle)));
    }


    public Command driveToHumanPlayer1() {
        return drivebase.driveToPose(
                new Pose2d(new Translation2d
                        (Meter.of(1),
                                Meter.of(7)),
                        Rotation2d.fromDegrees(130)));
    }

    public Command driveToProcessor() {
        return drivebase.driveToPose(
                new Pose2d(new Translation2d
                        (Meter.of(11.5),
                                Meter.of(7.5)),
                        Rotation2d.fromDegrees(90)));
    }

    public ParallelCommandGroup setElevArm(double goal, double degree) {
        return new ParallelCommandGroup(elevator.setGoal(goal), coralArm.setGoal(degree));
    }


}
