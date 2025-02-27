// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Meter;

import java.util.Set;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.Odometry;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.AlgaeArmConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.AlgaeArmSubsystem;
import frc.robot.subsystems.AlgaeIntakeSubsystem;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.CoralArmSubsystem;
import frc.robot.subsystems.CoralIntakeSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.FloorIntakeSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.systems.LoadingSystem;
import frc.robot.systems.ScoringSystem;
import frc.robot.systems.TargetingSystem;
import frc.robot.systems.field.AllianceFlipUtil;
import frc.robot.systems.field.FieldConstants;
import swervelib.SwerveController;
import swervelib.SwerveDrive;
import swervelib.SwerveInputStream;
import frc.robot.systems.field.FieldConstants.CoralStation;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer
{

  public static final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);
  // The robot's subsystems and commands are defined here...
  private final SwerveSubsystem       drivebase          = new SwerveSubsystem();
  // Replace with CommandPS4Controller or CommandJoystick if needed


  private final CommandXboxController m_OperatorController1 =
      new CommandXboxController(OperatorConstants.kOperatorControllerPort);
  private final CommandXboxController m_OperatorController2 =
      new CommandXboxController(OperatorConstants.kOperatorController2Port);
  private final CommandXboxController m_OperatorController3 =
      new CommandXboxController(OperatorConstants.kOperatorController3Port);

  private final ElevatorSubsystem    elevator    = new ElevatorSubsystem();
  private final CoralArmSubsystem    coralArm    = new CoralArmSubsystem();
  // private final ClimberSubsystem     climb       = new ClimberSubsystem();
  private final AlgaeIntakeSubsystem algaeIntake = new AlgaeIntakeSubsystem();
  private final AlgaeArmSubsystem    algaeArm    = new AlgaeArmSubsystem();
  private final FloorIntakeSubsystem floorIntake = new FloorIntakeSubsystem();
  private final CoralIntakeSubsystem coralIntake = new CoralIntakeSubsystem();

  private final TargetingSystem targetingSystem = new TargetingSystem();
  private final LoadingSystem   loadingSystem   = new LoadingSystem(coralArm, algaeArm, elevator, coralIntake, targetingSystem, algaeIntake);
  private final ScoringSystem   scoringSystem   = new ScoringSystem(coralArm,
                                                                    elevator,
                                                                    drivebase,
                                                                    algaeIntake,
                                                                    algaeArm,
                                                                    loadingSystem,
                                                                    targetingSystem,
                                                                    coralIntake);
  // The real world (whats that?)
  SwerveInputStream driveAngularVelocity = SwerveInputStream.of(drivebase.getSwerveDrive(),
                                                                () -> m_driverController.getLeftY() * -1,
                                                                () -> m_driverController.getLeftX() * -1)
                                                            .withControllerRotationAxis(m_driverController::getRightX)
                                                            .deadband(OperatorConstants.DEADBAND)
                                                            .scaleTranslation(0.8)
                                                            .allianceRelativeControl(true);

  SwerveInputStream driveDirectAngle = driveAngularVelocity.copy()
                                                           .withControllerHeadingAxis(m_driverController::getRightX,
                                                                                      m_driverController::getRightY)
                                                           .headingWhile(true);
  
  Command driveFieldOrientedDriectAngle = drivebase.driveFieldOriented(driveDirectAngle);
  Command driveSetpointGen = drivebase.driveWithSetpointGeneratorFieldRelative(driveDirectAngle);
  Command driveFieldOrientedAngularVelocity = drivebase.driveFieldOriented(driveAngularVelocity);

//Non reality code
  SwerveInputStream driveAngularVelocitySim = SwerveInputStream.of(drivebase.getSwerveDrive(),
                                                                   () -> -m_driverController.getLeftY(),
                                                                   () -> -m_driverController.getLeftX())
                                                               .withControllerRotationAxis(() -> m_driverController.getRawAxis(
                                                                   2))
                                                               .deadband(OperatorConstants.DEADBAND)
                                                               .scaleTranslation(0.8)
                                                               .allianceRelativeControl(true);



  // Derive the heading axis with math!
  SwerveInputStream driveDirectAngleSim     = driveAngularVelocitySim.copy()
                                                                     .withControllerHeadingAxis(() -> Math.sin(
                                                                                                    m_driverController.getRawAxis(
                                                                                                        2) * Math.PI) * (Math.PI * 2),
                                                                                                () -> Math.cos(
                                                                                                    m_driverController.getRawAxis(
                                                                                                        2) * Math.PI) *
                                                                                                      (Math.PI * 2))
                                                                     .headingWhile(true);
SwerveInputStream driveRobotOriented = driveAngularVelocity.copy().robotRelative(true)
                                                             .allianceRelativeControl(false);
  Command driveFieldOrientedDirectAngleSim = drivebase.driveFieldOriented(driveDirectAngleSim);

  Command driveSetpointGenSim = drivebase.driveWithSetpointGeneratorFieldRelative(driveDirectAngleSim);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  private double me = 0;

  /**
   * TO DO Change keyboard settings - add All pose in target system *get interns to do - Add all the buttons *get
   * interns to do
   * <p>
   * Fix Arm simulation, change angle of arm on start up, currently -90 -difference between the goal and the actual
   * location it reaches-because of consuming extra power?-pid tuning -it takes really long to move each arm????But all
   * three sim is working
   * <p>
   * set MAX MIN ANGLE for arm, ask cad team?-no limits (360 degrees Change height of elevator.max height of the barge.
   * currently in meters change that HEIGHT AND ANGLE FOR EACH LEVEL(ARM LENGTH?)-no idea yet
   * <p>
   * (Bumpers: 6 inch Elevator alone Min: 39.25 Unextended:41 1/2(from the ground)  Extended Elevator:71.094 so Second
   * half of the elevator:29.594 For the net: extended elev + algaeAngle(val?)
   * <p>
   * -start from horizontal(degrees) -L4 +80 `   Copy Math class Copy code from yagsl test code.
   * <p>
   * Add sensors -for algae, using algaeInArm.get()?
   * <p>
   * <p>
   * FIX QUESTION AND ASK LIMITS
   */

  public RobotContainer()
  {
    // Configure the trigger bindings
    DriverStation.silenceJoystickConnectionWarning(true);
     elevator.setDefaultCommand(elevator.setGoal(0));
    coralArm.setDefaultCommand(coralArm.setGoal(-45));
    //climb.setDefaultCommand(climb.stop());
    algaeIntake.setDefaultCommand(algaeIntake.setAlgaeIntakeRoller(0));
    algaeArm.setDefaultCommand(algaeArm.setGoal(-45));
    coralIntake.setDefaultCommand(coralIntake.spitCoralOut(0, 0));
    targetingSystem.setTarget(TargetingSystem.ReefBranch.A, TargetingSystem.ReefBranchLevel.L3);
    
//    floorIntake.setDefaultCommand(floorIntake.setCoralIntakeAngle(0));



driveDirectAngle
        .driveToPose(targetingSystem::getTargetPose,
                     new ProfiledPIDController(5, 0, 0, new TrapezoidProfile.Constraints(5, 2)),
                     new ProfiledPIDController(5, 0, 0, new TrapezoidProfile.Constraints(360, 180)));
    Command driveFieldOrientedDriectAngle = drivebase.driveFieldOriented(driveDirectAngle);

    drivebase.setDefaultCommand(driveFieldOrientedDriectAngle);
    SmartDashboard.putData(CommandScheduler.getInstance());

    boolean driveToPoseTesting = true;
    if (driveToPoseTesting)
    {
      m_driverController.button(1).whileTrue(Commands.startRun(() ->
                                                                   targetingSystem.autoTarget(drivebase::getPose),
                                                               () -> driveDirectAngle.driveToPoseEnabled(() -> !driveDirectAngle.atTargetPose(
                                                                   0.01))));
    }







//        targetingSystem.setTarget(ReefBranch.G,  ReefBranchLevel.L2);
        drivebase.getSwerveDrive().field.getObject("REEF").setPose(targetingSystem.getTargetPose());
     configureBindings();
    //drivebase.setDefaultCommand(driveFieldOrientedDriectAngle);
    SmartDashboard.putData(CommandScheduler.getInstance());

    // Elevator Testing
    boolean elevatorTesting = false;
    if(elevatorTesting)
    {
      m_driverController.button(1).whileTrue(elevator.setPower(0.1).until(elevator.atMax));
      m_driverController.button(2).whileTrue(elevator.runSysIdRoutine());
      m_driverController.button(3).whileTrue(elevator.setElevatorHeight(0.35).repeatedly());
      m_driverController.button(4).whileTrue(elevator.setElevatorHeight(0.1).repeatedly());
      m_driverController.button(5).whileTrue(elevator.setElevatorHeight(0.5).repeatedly());
      elevator.setDefaultCommand(elevator.hold());
    }

    boolean algaeArmTesting = false;
    if(algaeArmTesting)
    {
      m_driverController.button(1).whileTrue(algaeArm.setPower(0.1));
      m_driverController.button(2).whileTrue(algaeArm.runSysIdRoutine());
      m_driverController.button(3).whileTrue(algaeArm.setAlgaeArmAngle(0).repeatedly());
      m_driverController.button(4).whileTrue(algaeArm.setAlgaeArmAngle(-45).repeatedly());
      m_driverController.button(5).whileTrue(algaeArm.setAlgaeArmAngle(90).repeatedly());
      algaeArm.setDefaultCommand(algaeArm.hold());
    }

    boolean coralArmTesting = false;
    if(coralArmTesting)
    {
      m_driverController.button(1).whileTrue(coralArm.setPower(0.1));
      m_driverController.button(2).whileTrue(coralArm.runSysIdRoutine());
      m_driverController.button(3).whileTrue(coralArm.setCoralArmAngle(90).repeatedly());
      m_driverController.button(4).whileTrue(coralArm.setCoralArmAngle(50).repeatedly());
      m_driverController.button(5).whileTrue(coralArm.setCoralArmAngle(-30).repeatedly());

      coralArm.setDefaultCommand(coralArm.hold());
    }

    boolean wristTesting = false;
    if(wristTesting)
    {
      m_driverController.button(1).whileTrue(coralIntake.setWristPower(0.1));
      m_driverController.button(2).whileTrue(coralIntake.setWristAngle(90));
      m_driverController.button(3).whileTrue(coralIntake.setWristAngle(0));
      coralArm.setDefaultCommand(coralArm.setCoralArmAngle(0).repeatedly());
    }

    // drivebase.setDefaultCommand(
    //     !RobotBase.isSimulation() ? driveFieldOrientedAngularVelocity : driveFieldOrientedDirectAngleSim);
    NamedCommands.registerCommand("test", Commands.print("Hello World"));
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary predicate, or via the
   * named factories in {@link edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller PS4}
   * controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight joysticks}.
   */
  private void configureBindings()
  {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    // Put Mechanism 2d to SmartDashboard
    SmartDashboard.putData("Side View", Constants.sideRobotView);
    
    m_OperatorController1.button(1).onTrue(targetingSystem.setTargetCommand(
      TargetingSystem.ReefBranch.J,
      TargetingSystem.ReefBranchLevel.L1)
    .andThen(Commands.defer(()-> drivebase.driveToPose(targetingSystem.getTargetPose()), Set.of(drivebase)))
    .andThen(Commands.defer(scoringSystem::scoreCoral,  Set.of(elevator, algaeArm,coralArm,drivebase,coralIntake))));

    
    m_OperatorController1.button(2).onTrue(targetingSystem.setTargetCommand(
      TargetingSystem.ReefBranch.J,
      TargetingSystem.ReefBranchLevel.L2)
    .andThen(Commands.defer(()-> drivebase.driveToPose(targetingSystem.getTargetPose()), Set.of(drivebase)))
    .andThen(Commands.defer(scoringSystem::scoreCoral,  Set.of(elevator, algaeArm,coralArm,drivebase, coralIntake))));

    m_OperatorController1.button(3).onTrue(targetingSystem.setTargetCommand(
      TargetingSystem.ReefBranch.J,
      TargetingSystem.ReefBranchLevel.L3)
    .andThen(Commands.defer(()-> drivebase.driveToPose(targetingSystem.getTargetPose()), Set.of(drivebase)))
    .andThen(Commands.defer(scoringSystem::scoreCoral,  Set.of(elevator, algaeArm,coralArm,drivebase, coralIntake))));

    m_OperatorController1.button(4).onTrue(targetingSystem.setTargetCommand(
      TargetingSystem.ReefBranch.J,
      TargetingSystem.ReefBranchLevel.L4)
    .andThen(Commands.defer(()-> drivebase.driveToPose(targetingSystem.getTargetPose()), Set.of(drivebase)))
    .andThen(Commands.defer(scoringSystem::scoreCoral,  Set.of(elevator, algaeArm,coralArm,drivebase, coralIntake))));


    m_OperatorController1.button(5).onTrue(Commands.print("Left Side selected"));
    m_OperatorController1.button(6).onTrue(Commands.print("Right Side selected")); 

    m_OperatorController1.button(7).onTrue(Commands.print("Launch Command"));
    m_OperatorController1.button(8).onTrue(Commands.print("Cancel Selected Command"));

    m_OperatorController1.button(9).onTrue(Commands.print("Outtake Coral"));
    m_OperatorController1.button(10).onTrue(loadingSystem.coralLoad());// Maybe does work and we just dont see it????

  
    
    m_driverController.button(11).whileTrue(
      targetingSystem.autoTargetCommand(drivebase::getPose).andThen
      (Commands.runOnce(()->driveDirectAngle.driveToPoseEnabled(()->!driveDirectAngle.atTargetPose(0.01)))
      .andThen(Commands.waitUntil(()->driveDirectAngle.atTargetPose(0.01))
      .andThen((Commands.runOnce(()->driveDirectAngle.driveToPoseEnabled(false)))
    .andThen(loadingSystem.algaeLoad(42,14))))));

    m_driverController.button(12).whileTrue(
      targetingSystem.autoTargetCommand(drivebase::getPose).andThen
      (Commands.runOnce(()->driveDirectAngle.driveToPoseEnabled(()->!driveDirectAngle.atTargetPose(0.01)))
      .andThen(Commands.waitUntil(()->driveDirectAngle.atTargetPose(0.01))
      .andThen((Commands.runOnce(()->driveDirectAngle.driveToPoseEnabled(false)))
    .andThen(loadingSystem.algaeLoad(42,44))))));

    m_OperatorController1.button(13).onTrue(scoringSystem.scoreAlgaeNet()); //does not move elevator down
    m_OperatorController1.button(14).onTrue(scoringSystem.scoreAlgaeProcessor());

    m_OperatorController1.button(19).onTrue(loadingSystem.coralLock());


    m_OperatorController1.button(15).whileTrue(driveToHumanPlayer1().repeatedly());
    m_OperatorController1.button(16).whileTrue(driveToHumanPlayer2().repeatedly());
   
  //   m_OperatorController1.button(17).whileTrue( //Dance
  //   elevator.setElevatorHeight(Units.inchesToMeters(31)).repeatedly()
  //   .alongWith(waveArms(80, 20))
  //   .until(() ->coralArm.aroundAngle(80)).withTimeout(2)
  //   .andThen(waveArms(20, 80))
  //   .until(() ->algaeArm.aroundAngle(80)).withTimeout(2)
  //   .until(() -> elevator.aroundHeight((Units.inchesToMeters(32))))

  //  );

   
   m_driverController.button(17).whileTrue(
    targetingSystem.autoTargetCommand(drivebase::getPose)
              .andThen(Commands.defer(()-> drivebase.driveToPose(targetingSystem.getTargetPose()), Set.of(drivebase)))
              .andThen(Commands.defer(scoringSystem::scoreCoral,  Set.of(elevator, algaeArm,coralArm,drivebase))));


              if (Robot.isSimulation())
              {
                Pose2d target = new Pose2d(new Translation2d(1, 4),
                                           Rotation2d.fromDegrees(90));
                //drivebase.getSwerveDrive().field.getObject("targetPose").setPose(target);
                driveDirectAngleSim.driveToPose(() -> target,
                                                     new ProfiledPIDController(5,
                                                                               0,
                                                                               0,
                                                                               new Constraints(5, 2)),
                                                     new ProfiledPIDController(5,
                                                                               0,
                                                                               0,
                                                                               new Constraints(Units.degreesToRadians(360),
                                                                                               Units.degreesToRadians(180))
                                                     ));
                m_driverController.start().onTrue(Commands.runOnce(() -> drivebase.resetOdometry(new Pose2d(3, 3, new Rotation2d()))));
                m_driverController.button(1).whileTrue(drivebase.sysIdDriveMotorCommand());
                m_driverController.button(2).whileTrue(Commands.runEnd(() -> driveDirectAngleSim.driveToPoseEnabled(true),
                                                               () -> driveDirectAngleSim.driveToPoseEnabled(false)));
              }           
  // m_driverController.button(18).whileTrue(algaeArm.setAlgaeArmAngle(250).repeatedly().andThen(climb.climbUp()));
  // Button 19 is used just for testing loading sys

/*
 *     .andThen(waveArms(80, 20))
    .andThen(waveArms(20, 80))
    .andThen(waveArms(80, 20))
    .andThen(waveArms(20, 80))
    .andThen(waveArms(20, 80))
    .andThen(waveArms(90, 90))
    .alongWith(drivebase.driveToPose(new Pose2d(new Translation2d(),
                                                                  drivebase.getRotation()
                                                                .minus(Rotation2d.fromDegrees(180)))))
  );
 */
   

      
  }


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand()
  {
    // An example command will be run in autonomous
    return drivebase.getAutonomousCommand("New Auto");
  }

  public Command driveToSetPoint(double x, double y, double angle)
  {
    return drivebase.driveToPose(
        new Pose2d(new Translation2d
                       (Meter.of(x),
                        Meter.of(y)),
                   Rotation2d.fromDegrees(angle)));
  }


  public Command driveToHumanPlayer1()
  {
    if (AllianceFlipUtil.shouldFlip()){
        return drivebase.driveToPose(AllianceFlipUtil.flip(CoralStation.leftCenterFace));
    } else  {
        return drivebase.driveToPose((CoralStation.leftCenterFace));
    }
  }
  public Command driveToHumanPlayer2()
  {
    if (AllianceFlipUtil.shouldFlip()){
      return drivebase.driveToPose(AllianceFlipUtil.flip(CoralStation.rightCenterFace));
  } else  {
      return drivebase.driveToPose((CoralStation.rightCenterFace));
  }
  }

  public Command driveToProcessor()
  {
    return drivebase.driveToPose(
        new Pose2d(new Translation2d
                       (Meter.of(11.5),
                        Meter.of(7.5)),
                   Rotation2d.fromDegrees(90)));
  }

  public ParallelCommandGroup setElevArm(double goal, double degree)
  {
    return new ParallelCommandGroup(elevator.setGoal(goal), coralArm.setGoal(degree));
  }

  public ParallelCommandGroup waveArms(double coralAngle, double algaeAngle){
    return new ParallelCommandGroup(coralArm.setGoal(coralAngle), algaeArm.setGoal(algaeAngle));
  }


  

}
