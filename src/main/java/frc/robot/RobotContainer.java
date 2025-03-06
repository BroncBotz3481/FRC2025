// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Meter;

import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.controllers.Launchpad;
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
import frc.robot.systems.TargetingSystem.ReefBranchLevel;
import frc.robot.systems.field.AllianceFlipUtil;
import frc.robot.systems.field.FieldConstants.CoralStation;
import swervelib.SwerveInputStream;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer
{

  public static final CommandXboxController m_driverController    =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);
  public static final CommandXboxController m_OperatorController1 =
      new CommandXboxController(OperatorConstants.kOperatorControllerPort);

  private final Launchpad launchpad = new Launchpad(1, 2, 3, new Color8Bit(Color.kRed));

  // The robot's subsystems and commands are defined here...
  private final SwerveSubsystem drivebase = new SwerveSubsystem();
  // Replace with CommandPS4Controller or CommandJoystick if needed


  // The real world (whats that?)
  SwerveInputStream driveAngularVelocity = SwerveInputStream.of(drivebase.getSwerveDrive(),
                                                                () -> m_driverController.getLeftY() * -1,
                                                                () -> m_driverController.getLeftX() * -1)
                                                            .withControllerRotationAxis(m_driverController::getRightX)
                                                            .deadband(OperatorConstants.DEADBAND)
                                                            .scaleTranslation(0.8)
                                                            .scaleRotation(0.4)
                                                            .allianceRelativeControl(true);

  private final ElevatorSubsystem    elevator    = new ElevatorSubsystem();
  private final CoralArmSubsystem    coralArm    = new CoralArmSubsystem();
  private final ClimberSubsystem     climb       = new ClimberSubsystem();
  private final AlgaeIntakeSubsystem algaeIntake = new AlgaeIntakeSubsystem();
  private final AlgaeArmSubsystem    algaeArm    = new AlgaeArmSubsystem();
  private final FloorIntakeSubsystem floorIntake = new FloorIntakeSubsystem();
  private final CoralIntakeSubsystem coralIntake = new CoralIntakeSubsystem();

  private final TargetingSystem targetingSystem = new TargetingSystem();
  private final LoadingSystem   loadingSystem   = new LoadingSystem(coralArm,
                                                                    algaeArm,
                                                                    elevator,
                                                                    coralIntake,
                                                                    targetingSystem,
                                                                    algaeIntake,
                                                                    drivebase);
  private final ScoringSystem   scoringSystem   = new ScoringSystem(coralArm,
                                                                    elevator,
                                                                    drivebase,
                                                                    algaeIntake,
                                                                    algaeArm,
                                                                    loadingSystem,
                                                                    targetingSystem,
                                                                    coralIntake,
                                                                    driveAngularVelocity);


  Command driveRobotOrientedAngularVelocity = drivebase.drive(driveAngularVelocity);

  //Non reality code
  SwerveInputStream driveAngularVelocitySim = SwerveInputStream.of(drivebase.getSwerveDrive(),
                                                                   () -> -m_driverController.getLeftY(),
                                                                   () -> -m_driverController.getLeftX())
                                                               .withControllerRotationAxis(() -> m_driverController.getRawAxis(
                                                                   2))
                                                               .deadband(OperatorConstants.DEADBAND)
                                                               .scaleTranslation(0.8)
                                                               .allianceRelativeControl(true);

  //INISTALIZING SUBSYSTEMS AND COMMANDS ^
//--------------------------------------------------------------------------------------------------------------------------------------- 
  //DEFUALT COMMANDS v

  public void setDefaultCommands()
  {
    elevator.setDefaultCommand(elevator.setPower(0));
    algaeArm.setDefaultCommand(algaeArm.setPower(0));
    coralArm.setDefaultCommand(coralArm.setPower(0));
    coralIntake.setDefaultCommand(coralIntake.wristRest());
  }

  //DEFAULT COMMANDS ^
//---------------------------------------------------------------------------------------------------------------------------------------
  //ROBOT CONTAINER


  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */


  public RobotContainer()
  {
    // Put Mechanism 2d to SmartDashboard
    SmartDashboard.putData("Side View", Constants.sideRobotView);
    // Configure the trigger bindings
    DriverStation.silenceJoystickConnectionWarning(true);
    // configureBindings();
    drivebase.setDefaultCommand(driveRobotOrientedAngularVelocity);
    SmartDashboard.putData(CommandScheduler.getInstance());

    setDefaultCommands();

//------------------------------------------------------------------------
    //TESTING COMMANDS v

    boolean scoreCoralTesting = false;
    if (scoreCoralTesting)
    {
      m_driverController.button(1).whileTrue(targetingSystem.autoTargetCommand(drivebase::getPose)
                                                            .andThen(targetingSystem.setBranchLevel(ReefBranchLevel.L3))
                                                            .andThen(scoringSystem.scoreCoral()));

      m_driverController.button(2).whileTrue(targetingSystem.setBranchLevel(ReefBranchLevel.L3)
                                                            .andThen(elevator.getCoralCommand(targetingSystem)
                                                                             .repeatedly()));
    }

    // Elevator Testing
    boolean elevatorTesting = false;
    if (elevatorTesting)
    {
      m_driverController.y().whileTrue(elevator.setPower(0.2).until(elevator.atMax));
      m_driverController.x().whileTrue(elevator.setPower(-0.2).until(elevator.atMax));
      m_OperatorController1.x().whileTrue(elevator.CoralL4()); // l4
      m_OperatorController1.y().whileTrue(elevator.CoralL3()); // l3
      m_OperatorController1.povRight().whileTrue(elevator.AlgaeL23()); // l2 algae
      m_OperatorController1.povLeft().whileTrue(elevator.AlgaeL34()); // l3 algae
      m_OperatorController1.start().whileTrue(elevator.CoralL2()); // l2
      m_OperatorController1.povLeft().whileTrue(elevator.AlgaeNET()); // barge

      // m_driverController.button(2).whileTrue(elevator.runSysIdRoutine());
      // m_driverController.button(3).whileTrue(elevator.setElevatorHeight(0.35).repeatedly());
      // m_driverController.button(4).whileTrue(elevator.setElevatorHeight(0.1).repeatedly());
      // m_driverController.button(5).whileTrue(elevator.setElevatorHeight(0.5).repeatedly());
      elevator.setDefaultCommand(elevator.hold());
    }

    boolean algaeArmTesting = false;
    if (algaeArmTesting)
    {
      m_driverController.b().whileTrue(algaeArm.setPower(0.2));
      m_driverController.a().whileTrue(algaeArm.setPower(-0.2));
      m_driverController.povLeft().whileTrue(algaeArm.setGoal(33.2).andThen(Commands.waitSeconds(2))
                                                     .andThen(algaeArm.setGoal(35))); // l3 algae
      m_driverController.povRight().whileTrue(algaeArm.setGoal(2.637).andThen(Commands.waitSeconds(2))
                                                      .andThen(algaeArm.setGoal(8))); // l2 algae
      m_OperatorController1.povLeft().whileTrue(algaeArm.setGoal(90)); // barge

      // m_driverController.button(2).whileTrue(algaeArm.runSysIdRoutine());
      // m_driverController.button(3).whileTrue(algaeArm.setAlgaeArmAngle(0).repeatedly());
      // m_driverController.button(4).whileTrue(algaeArm.setAlgaeArmAngle(-45).repeatedly());
      // m_driverController.button(5).whileTrue(algaeArm.setAlgaeArmAngle(90).repeatedly());
      m_driverController.leftBumper().whileTrue(algaeIntake.setAlgaeIntakeRoller(0.8));
      m_driverController.rightBumper().whileTrue(algaeIntake.setAlgaeIntakeRoller(-0.8));

      algaeIntake.setDefaultCommand(algaeIntake.setAlgaeIntakeRoller(0));
      algaeArm.setDefaultCommand(algaeArm.hold());
    }

    boolean coralArmTesting = false;
    if (coralArmTesting)
    {
      m_driverController.povUp().whileTrue(coralArm.setPower(0.1));
      m_driverController.povDown().whileTrue(coralArm.setPower(-0.1));
      m_OperatorController1.x().whileTrue(coralArm.setGoal(57.9)); // l4
      m_OperatorController1.y().whileTrue(coralArm.setGoal(36.14)); // l3
      m_OperatorController1.start().whileTrue(coralArm.setGoal(10)); // l2
      m_OperatorController1.povDown().whileTrue(coralArm.score());

      // m_driverController.button(2).whileTrue(coralArm.runSysIdRoutine());
      // m_driverController.button(3).whileTrue(coralArm.setCoralArmAngle(90).repeatedly());
      // m_driverController.button(4).whileTrue(coralArm.setCoralArmAngle(50).repeatedly());
      // m_driverController.button(5).whileTrue(coralArm.setCoralArmAngle(-30).repeatedly());
      m_OperatorController1.leftBumper().whileTrue(coralIntake.setCoralIntakePower(0.5));
      m_OperatorController1.rightBumper().whileTrue(coralIntake.setCoralIntakePower(-0.2));

      coralIntake.setDefaultCommand(coralIntake.setCoralIntakePower(0));
      coralArm.setDefaultCommand(coralArm.hold());
    }

    boolean wristTesting = false;
    if (wristTesting)
    {
      m_driverController.a().whileTrue(coralIntake.setWristPower(0.1));
      m_driverController.y().whileTrue(coralIntake.setWristPower(-0.1));

      m_driverController.b().whileTrue(coralIntake.wristIntake());
      m_driverController.x().whileTrue(coralIntake.wristOuttake());
      coralArm.setDefaultCommand(coralArm.setCoralArmAngle(0).repeatedly());
      coralIntake.setDefaultCommand(coralIntake.wristRest());
    }

    NamedCommands.registerCommand("test", Commands.print("Hello World"));
  }

  //ROBOT CONTAINER ^
//---------------------------------------------------------------------------------------------------------------------------------------
  //CONFIGURE BINDINGS v

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary predicate, or via the
   * named factories in {@link edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller PS4}
   * controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight joysticks}.
   */
  private void configureBindings()
  {

    m_driverController.povUp().whileTrue(climb.climbUp());
    m_driverController.povDown().whileTrue(climb.climbDown());

    m_driverController.leftBumper()
                      .whileTrue(Commands.run(() -> driveAngularVelocity.scaleTranslation(0.4))); // Slow mode
    m_driverController.leftBumper()
                      .whileFalse(Commands.run(() -> driveAngularVelocity.scaleTranslation(0.8)));//Fast mode

    m_driverController.y().onTrue(Commands.print("Turn 90 Clockwise"));
    m_driverController.x().onTrue(Commands.print("Turn 90 Counter-Clockwise"));

    //DRIVER CONTROLS ^
//--------------------------------------------------------------------------------------------------------------------------------------
    //OPERATOR CONTROLS - Launchpad v

    boolean launchpadTesting = false;
    if (launchpadTesting)
    {
      //L1 Score Coral
      launchpad.getButton(1, 0).whileTrue(targetingSystem.autoTargetCommand(drivebase::getPose)
                                                         .andThen(targetingSystem.setBranchLevel(ReefBranchLevel.L1))
                                                         .andThen(scoringSystem.scoreCoral()));
      //L2 Score Coral
      launchpad.getButton(2, 0).whileTrue(targetingSystem.autoTargetCommand(drivebase::getPose)
                                                         .andThen(targetingSystem.setBranchLevel(ReefBranchLevel.L2))
                                                         .andThen(scoringSystem.scoreCoral()));
      //L3 Score Coral
      launchpad.getButton(3, 0).whileTrue(targetingSystem.autoTargetCommand(drivebase::getPose)
                                                         .andThen(targetingSystem.setBranchLevel(ReefBranchLevel.L3))
                                                         .andThen(scoringSystem.scoreCoral()));
      //L4 Score Coral
      launchpad.getButton(4, 0).whileTrue(targetingSystem.autoTargetCommand(drivebase::getPose)
                                                         .andThen(targetingSystem.setBranchLevel(ReefBranchLevel.L4))
                                                         .andThen(scoringSystem.scoreCoral()));

      //Algae Load L23
      launchpad.getButton(1, 2).whileTrue(targetingSystem.autoTargetCommand(drivebase::getPose)
                                                         .andThen(targetingSystem.setBranchLevel(ReefBranchLevel.L2))
                                                         .andThen(loadingSystem.algaeLoad()));

      //Algae Load L34
      launchpad.getButton(1, 3).whileTrue(targetingSystem.autoTargetCommand(drivebase::getPose)
                                                         .andThen(targetingSystem.setBranchLevel(ReefBranchLevel.L3))
                                                         .andThen(loadingSystem.algaeLoad()));

      //Score Net
      launchpad.getButton(0, 0).whileTrue(scoringSystem.scoreAlgaeNet());

      //Score Processor
      launchpad.getButton(1, 0).whileTrue(scoringSystem.scoreAlgaeProcessor());

      // m_OperatorController1.button(19).onTrue(loadingSystem.coralLock());

      launchpad.getButton(8, 1).whileTrue(loadingSystem.coralLoadLeft());
      launchpad.getButton(8, 2).whileTrue(loadingSystem.coralLoadRight());

      //LAUNCH PAD ^
    } else
    { //--------------------------------------------------------------------------------------------------------------------
      //SIM BUTTON CONTROLS v

      //L1 Score Coral
      m_OperatorController1.button(1).whileTrue(targetingSystem.autoTargetCommand(drivebase::getPose)
                                                               .andThen(targetingSystem.setBranchLevel(ReefBranchLevel.L1))
                                                               .andThen(scoringSystem.scoreCoral()));
      //L2 Score Coral
      m_OperatorController1.button(2).whileTrue(targetingSystem.autoTargetCommand(drivebase::getPose)
                                                               .andThen(targetingSystem.setBranchLevel(ReefBranchLevel.L2))
                                                               .andThen(scoringSystem.scoreCoral()));
      //L3 Score Coral
      m_OperatorController1.button(3).whileTrue(targetingSystem.autoTargetCommand(drivebase::getPose)
                                                               .andThen(targetingSystem.setBranchLevel(ReefBranchLevel.L3))
                                                               .andThen(scoringSystem.scoreCoral()));
      //L4 Score Coral
      m_OperatorController1.button(4).whileTrue(targetingSystem.autoTargetCommand(drivebase::getPose)
                                                               .andThen(targetingSystem.setBranchLevel(ReefBranchLevel.L3))
                                                               .andThen(scoringSystem.scoreCoral()));

      
      //Algae Load L23
      m_OperatorController1.button(11).whileTrue(targetingSystem.autoTargetCommand(drivebase::getPose)
                                                                .andThen(targetingSystem.setBranchLevel(ReefBranchLevel.L2))
                                                                .andThen(loadingSystem.algaeLoad()));

      //Algae Load L34
      m_OperatorController1.button(12).whileTrue(targetingSystem.autoTargetCommand(drivebase::getPose)
                                                                .andThen(targetingSystem.setBranchLevel(ReefBranchLevel.L3))
                                                                .andThen(loadingSystem.algaeLoad()));

      //Score Net
      m_OperatorController1.button(13).onTrue(scoringSystem.scoreAlgaeNet());

      //Score Processor
      m_OperatorController1.button(14).onTrue(scoringSystem.scoreAlgaeProcessor());

      m_OperatorController1.button(19).onTrue(loadingSystem.coralLock());

      m_OperatorController1.button(15).whileTrue(loadingSystem.coralLoadLeft());
      m_OperatorController1.button(16).whileTrue(loadingSystem.coralLoadRight());

    }
  }
//END OF CONFIG BINDINGS ^
//----------------------------------------------------------------------------------------------------------------------------------
// MISC METHODS AND COMMANDS v

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
    if (AllianceFlipUtil.shouldFlip())
    {
      return drivebase.driveToPose(AllianceFlipUtil.flip(CoralStation.leftCenterFace));
    } else
    {
      return drivebase.driveToPose((CoralStation.leftCenterFace));
    }
  }

  public Command driveToHumanPlayer2()
  {
    if (AllianceFlipUtil.shouldFlip())
    {
      return drivebase.driveToPose(AllianceFlipUtil.flip(CoralStation.rightCenterFace));
    } else
    {
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

  public ParallelCommandGroup waveArms(double coralAngle, double algaeAngle)
  {
    return new ParallelCommandGroup(coralArm.setGoal(coralAngle), algaeArm.setGoal(algaeAngle));
  }


}
