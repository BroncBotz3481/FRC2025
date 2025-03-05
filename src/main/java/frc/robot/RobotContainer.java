// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Meter;

import java.util.Set;

import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.Odometry;
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
  public static final CommandXboxController m_OperatorController1 =
      new CommandXboxController(OperatorConstants.kOperatorControllerPort);
  // The robot's subsystems and commands are defined here...
  private final SwerveSubsystem       drivebase          = new SwerveSubsystem();
  // Replace with CommandPS4Controller or CommandJoystick if needed


  // The real world (whats that?)
  SwerveInputStream driveAngularVelocity = SwerveInputStream.of(drivebase.getSwerveDrive(),
                                                                () -> m_driverController.getLeftY() * -1,
                                                                () -> m_driverController.getLeftX() * -1)
                                                            .withControllerRotationAxis(m_driverController::getRightX)
                                                            .deadband(OperatorConstants.DEADBAND)
                                                            .scaleTranslation(0.8)
                                                            .scaleRotation(0.4)
                                                            .allianceRelativeControl(false);

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
    // configureBindings();
    drivebase.setDefaultCommand(driveRobotOrientedAngularVelocity);
    SmartDashboard.putData(CommandScheduler.getInstance());

    // Elevator Testing
    boolean elevatorTesting = false;
    if(elevatorTesting)
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
    if(algaeArmTesting)
    {
      m_driverController.b().whileTrue(algaeArm.setPower(0.2));
      m_driverController.a().whileTrue(algaeArm.setPower(-0.2));
      m_driverController.povLeft().whileTrue(algaeArm.setGoal(33.2).andThen(Commands.waitSeconds(2)).andThen(algaeArm.setGoal(35))); // l3 algae
      m_driverController.povRight().whileTrue(algaeArm.setGoal(2.637).andThen(Commands.waitSeconds(2)).andThen(algaeArm.setGoal(8))); // l2 algae
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
    if(coralArmTesting)
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

    boolean wristTesting = true;
    if(wristTesting)
    {
      m_driverController.a().whileTrue(coralIntake.setWristPower(0.1));
      m_driverController.y().whileTrue(coralIntake.setWristPower(-0.1));

      m_driverController.b().whileTrue(coralIntake.wristIntake());
      m_driverController.x().whileTrue(coralIntake.wristOuttake());
      coralArm.setDefaultCommand(coralArm.setCoralArmAngle(0).repeatedly());
      coralIntake.setDefaultCommand(coralIntake.wristRest());
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
    
    m_OperatorController1.button(1).onTrue(Commands.print("Level 1 selected"));
    m_OperatorController1.button(2).onTrue(Commands.print("Level 2 selected"));
    m_OperatorController1.button(3).onTrue(Commands.print("Level 3 selected"));
    m_OperatorController1.button(4).onTrue(Commands.print("Level 4 selected"));

    m_OperatorController1.button(5).onTrue(Commands.print("Left Side selected"));
    m_OperatorController1.button(6).onTrue(Commands.print("Right Side selected")); 

    m_OperatorController1.button(7).onTrue(Commands.print("Launch Command"));
    m_OperatorController1.button(8).onTrue(Commands.print("Cancel Selected Command"));

    m_OperatorController1.button(9).onTrue(Commands.print("Outtake Coral"));
    m_OperatorController1.button(10).onTrue(loadingSystem.coralLoad());// Maybe does work and we just dont see it????

  
    m_OperatorController1.button(11).onTrue( 
      targetingSystem.setTargetCommand(
        TargetingSystem.ReefBranch.J, //I just need the height of the levels, not the specific branch, how to do that
        TargetingSystem.ReefBranchLevel.L2).andThen(loadingSystem.algaeLoad(42, 14)));
        
    m_OperatorController1.button(12).onTrue( 
      targetingSystem.setTargetCommand(
        TargetingSystem.ReefBranch.J, //I just need the height of the levels, not the specific branch, how to do that
        TargetingSystem.ReefBranchLevel.L3).andThen(loadingSystem.algaeLoad(42, 44)));

    m_OperatorController1.button(13).onTrue(scoringSystem.scoreAlgaeNet()); //does not move elevator down
    m_OperatorController1.button(14).onTrue(scoringSystem.scoreAlgaeProcessor());

    m_OperatorController1.button(19).onTrue(loadingSystem.coralLock());


    m_OperatorController1.button(15).whileTrue(driveToHumanPlayer1().repeatedly());
    m_OperatorController1.button(16).whileTrue(driveToHumanPlayer2().repeatedly());


   
   m_driverController.button(17).whileTrue(
    targetingSystem.autoTargetCommand(drivebase::getPose)
              .andThen(Commands.defer(()-> drivebase.driveToPose(targetingSystem.getTargetPose()), Set.of(drivebase)))
              .andThen(Commands.defer(scoringSystem::scoreCoral,  Set.of(elevator, algaeArm,coralArm,drivebase))));

  }

  private void changeMe()
  {
    me = 1;
  }

  private double getMe()
  {
    return me;
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
