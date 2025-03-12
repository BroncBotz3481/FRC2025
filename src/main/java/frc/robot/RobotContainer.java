// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meter;

import com.pathplanner.lib.auto.NamedCommands;
import com.reduxrobotics.canand.CanandEventLoop;

import au.grapplerobotics.CanBridge;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.CoralArmConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Setpoints.AutoScoring.HumanPlayer;
import frc.robot.Setpoints.AutoScoring.HumanPlayer.Left;
import frc.robot.controllers.ButtonColours;
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
import frc.robot.systems.TargetingSystem.ReefBranch;
import frc.robot.systems.TargetingSystem.ReefBranchLevel;
import frc.robot.systems.TargetingSystem.ReefBranchSide;
import frc.robot.systems.field.AllianceFlipUtil;
import frc.robot.systems.field.FieldConstants;
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
                                                                () -> m_driverController.getLeftY() ,
                                                                () -> m_driverController.getLeftX())
                                                            .withControllerRotationAxis(()->m_driverController.getRightX()*-1)
                                                            .deadband(OperatorConstants.DEADBAND)
                                                            .scaleTranslation(0.8)
                                                            .scaleRotation(0.6)
                                                            .allianceRelativeControl(true);
  SwerveInputStream driveDirectAngle = driveAngularVelocity.copy().withControllerHeadingAxis(()->m_driverController.getRightX(),()->m_driverController.getRightY())
  .headingWhile(true);

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
                                                                    climb,
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

  public void fixCoralArmAngle()
  {
    double coralAngle = coralArm.getAngle().in(Degrees);
    if(MathUtil.clamp(coralAngle, CoralArmConstants.kCoralArmMinAngle.in(Degrees), -35) != coralAngle)
    {
      coralArm.setAngleEncoderPosition(CoralArmConstants.kCoralArmStartingAngle);
    }

  }
  public void setDefaultCommands()
  {
    // Sets the coral arm angle to the same as the algae arm if it is not starting in the right height
    //fixCoralArmAngle();

    elevator.setDefaultCommand((elevator.setGoal(0.010)));
    algaeArm.setDefaultCommand(algaeArm.hold());
    coralArm.setDefaultCommand(coralArm.hold());
    climb.setDefaultCommand(climb.setPOwer(0));
    coralIntake.setDefaultCommand(coralIntake.setCoralIntakePower(0));
    algaeIntake.setDefaultCommand(algaeIntake.hold(()->algaeArm.algaeLoaded() && algaeArm.getAngle().gte(Degrees.of(-30))));
    coralIntake.setDefaultCommand(coralIntake.hold(coralArm::coralLoaded));
    // coralIntake.setDefaultCommand(Commands.either(coralIntake.wristIntake(), coralIntake.wristRest(), coralArm::coralLoaded));
    // algaeIntake.setDefaultCommand(Commands.either(algaeIntake.in(), algaeIntake.setAlgaeIntakeRoller(0), algaeArm::algaeLoaded));
    // drivebase.setDefaultCommand(drivebase.driveFieldOriented(driveDirectAngle));
    drivebase.setDefaultCommand(driveRobotOrientedAngularVelocity);

    launchpad.changeLED(2,0, new Color8Bit(ButtonColours.AlgaeColour));
    launchpad.changeLED(0,1, new Color8Bit(ButtonColours.CoralLevelColour));
    launchpad.changeLED(0,2, new Color8Bit(ButtonColours.CoralLevelColour));
    launchpad.changeLED(0,3, new Color8Bit(ButtonColours.CoralLevelColour));
    launchpad.changeLED(0,4, new Color8Bit(ButtonColours.CoralLevelColour));
    launchpad.changeLED(0,5, new Color8Bit(ButtonColours.notSelectedColour));
    launchpad.changeLED(0,6, new Color8Bit(ButtonColours.notSelectedColour));
    launchpad.changeLED(0,7, new Color8Bit(ButtonColours.notSelectedColour));
    launchpad.changeLED(0,8, new Color8Bit(ButtonColours.notSelectedColour));

    launchpad.changeLED(1,0, new Color8Bit(ButtonColours.AlgaeColour));
    launchpad.changeLED(1,2, new Color8Bit(ButtonColours.AlgaeColour));
    launchpad.changeLED(1,3, new Color8Bit(ButtonColours.AlgaeColour));
    launchpad.changeLED(1,7, new Color8Bit(ButtonColours.AlgaeUnloaded));
    launchpad.changeLED(1,8, new Color8Bit(ButtonColours.CoralUnloaded));

    launchpad.changeLED(3, 0, new Color8Bit(Color.kRed));
    launchpad.changeLED(4, 0, new Color8Bit(Color.kBlue));


    launchpad.changeLED(7,1, new Color8Bit(ButtonColours.HP));

    launchpad.changeLED(7,8, new Color8Bit(ButtonColours.ScoreCoral));

    launchpad.changeLED(8, 1, new Color8Bit(Color.kLimeGreen));
    launchpad.changeLED(8, 2, new Color8Bit(Color.kMediumPurple));
  }

  //DEFAULT COMMANDS ^
//---------------------------------------------------------------------------------------------------------------------------------------
  //ROBOT CONTAINER


  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */


  public RobotContainer()
  {
    CanandEventLoop.getInstance();
    CanBridge.runTCP();
    // Put Mechanism 2d to SmartDashboard
    SmartDashboard.putData("Side View", Constants.sideRobotView);
    // Configure the trigger bindings
     setDefaultCommands();
     configureBindings();

    SmartDashboard.putData(CommandScheduler.getInstance());


//------------------------------------------------------------------------
    //TESTING COMMANDS v

    boolean autoPoseFetchTest = false;
    if(autoPoseFetchTest)
    {

      drivebase.setDefaultCommand(driveRobotOrientedAngularVelocity);
      m_driverController.button(1).whileTrue(Commands.runOnce(()->targetingSystem.printTargetPose(ReefBranch.A, ReefBranchLevel.L4, ReefBranchSide.CLOSEST))
                                                     .andThen(Commands.runOnce(()->targetingSystem.setCoralTargetOnField(drivebase))));
      m_driverController.button(2).whileTrue(Commands.runOnce(()->{
                                                       targetingSystem.increaseBranch();
                                                       targetingSystem.printTargetPose(ReefBranchLevel.L4, ReefBranchSide.CLOSEST);})
                                                     .andThen(Commands.runOnce(()->targetingSystem.setCoralTargetOnField(drivebase))));

    }

    boolean climberTesting = false;
    if(climberTesting)
    {
      m_driverController.y().whileTrue(climb.up());
      m_driverController.x().whileTrue(climb.down());
      m_driverController.b().whileTrue(climb.setPOwer(0.5));
      m_driverController.a().whileTrue(climb.setPOwer(-0.5));
      m_driverController.leftBumper().whileTrue(loadingSystem.dislodgeAlgaeArm());
      climb.setDefaultCommand(climb.setPOwer(0));

    }

    boolean armSensorTesting = false;
    if(armSensorTesting)
    {
      algaeArm.setDefaultCommand(algaeArm.setAlgaeArmAngle(0));
      coralArm.setDefaultCommand(coralArm.setCoralArmAngle(0));
      algaeIntake.setDefaultCommand(algaeIntake.setAlgaeIntakeRoller(0));
      coralIntake.setDefaultCommand(coralIntake.wristRest());
      m_driverController.x().whileTrue(algaeIntake.in());
      m_driverController.y().whileTrue(algaeIntake.out());
      m_driverController.a().whileTrue(coralIntake.wristIntake());
      m_driverController.b().whileTrue(coralIntake.wristOuttake());
    }

    boolean scoreCoralTesting = false;
    if (scoreCoralTesting)
    {
      m_driverController.a().whileTrue(coralArm.setPower(0.1));
      m_driverController.b().whileTrue(coralArm.setPower(-0.1));

      m_driverController.x().whileTrue(targetingSystem.autoTargetCommand(drivebase::getPose)
                                                            .andThen(targetingSystem.setBranchLevel(ReefBranchLevel.L3))
                                                            .andThen(scoringSystem.scoreCoral()));

      m_driverController.y().whileTrue(targetingSystem.autoTargetCommand(drivebase::getPose)
                                                            .andThen(targetingSystem.setBranchLevel(ReefBranchLevel.L2))
                                                            .andThen(scoringSystem.scoreCoral()));
      m_driverController.leftBumper().whileTrue(targetingSystem.autoTargetCommand(drivebase::getPose)
      .andThen(targetingSystem.setBranchLevel(ReefBranchLevel.L4))
      .andThen(scoringSystem.scoreCoral()));
      m_driverController.rightBumper().whileTrue(coralIntake.wristIntake());

      elevator.setDefaultCommand(elevator.hold());
      coralArm.setDefaultCommand(coralArm.hold());
      coralIntake.setDefaultCommand(Commands.either(coralIntake.wristIntake(), coralIntake.wristRest(), coralArm::coralLoaded));
    }

    // Elevator Testing
    boolean elevatorTesting = false;
    if (elevatorTesting)
    {
      m_driverController.leftBumper().whileTrue(elevator.setPower(0.2).until(elevator.atMax));
      m_driverController.rightBumper().whileTrue(elevator.setPower(-0.2).until(elevator.atMax));
      m_driverController.x().whileTrue(elevator.CoralL4()); // l4
      m_driverController.y().whileTrue(elevator.CoralL3()); // l3
      m_driverController.povRight().whileTrue(elevator.AlgaeL23().repeatedly()); // l2 algae
      m_driverController.povLeft().whileTrue(elevator.AlgaeL34().repeatedly()); // l3 algae
      m_driverController.start().whileTrue(elevator.CoralL2().repeatedly()); // l2
      m_driverController.povLeft().whileTrue(elevator.AlgaeNET().repeatedly()); // barge

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
      // m_driverController.povLeft().whileTrue(algaeArm.setAlgaeArmAngle(33.2).andThen(Commands.waitSeconds(2))
      //                                                .andThen(algaeArm.setAlgaeArmAngle(35))); // l3 algae
      // m_driverController.povRight().whileTrue(algaeArm.setAlgaeArmAngle(2.637).andThen(Commands.waitSeconds(2))
      //                                                 .andThen(algaeArm.setAlgaeArmAngle(8))); // l2 algae
      // m_driverController.povLeft().whileTrue(algaeArm.setAlgaeArmAngle(90)); // barge
      m_driverController.povUp().whileTrue(targetingSystem.setBranchLevel(ReefBranchLevel.L3).andThen(targetingSystem.autoTargetCommand(drivebase::getPose)).andThen(loadingSystem.algaeLoad()).andThen(algaeIntake.in()));
      m_driverController.povDown().whileTrue(targetingSystem.setBranchLevel(ReefBranchLevel.L2).andThen(targetingSystem.autoTargetCommand(drivebase::getPose)).andThen(loadingSystem.algaeLoad()).andThen(algaeIntake.in()));
      
      m_driverController.y().whileTrue(algaeArm.L34());
      m_driverController.rightBumper().whileTrue(algaeIntake.out());

      // m_driverController.x().whileTrue(algaeArm.runSysIdRoutine());
      // m_driverController.y().whileTrue(algaeArm.setAlgaeArmAngle(0).repeatedly());
      // m_driverController.start().whileTrue(algaeArm.setAlgaeArmAngle(-45).repeatedly());
      // m_driverController.povLeft().whileTrue(algaeArm.setAlgaeArmAngle(90).repeatedly());
      // m_driverController.leftBumper().whileTrue(algaeIntake.setAlgaeIntakeRoller(0.8));
      // m_driverController.rightBumper().whileTrue(algaeIntake.setAlgaeIntakeRoller(-0.8));

      // algaeIntake.setDefaultCommand(algaeIntake.setAlgaeIntakeRoller(0));
      algaeArm.setDefaultCommand(algaeArm.hold());
    }

    boolean coralArmTesting = false;
    if (coralArmTesting)
    {
      m_driverController.povUp().whileTrue(coralArm.setPower(0.1));
      m_driverController.povDown().whileTrue(coralArm.setPower(-0.1));
      m_driverController.x().whileTrue(coralArm.setCoralArmAngle(57.9)); // l4
      m_driverController.y().whileTrue(coralArm.setCoralArmAngle(36.14)); // l3
      m_OperatorController1.start().whileTrue(coralArm.setGoal(10)); // l2
      m_driverController.a().whileTrue(coralArm.setCoralArmAngle(90)); 
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

    NamedCommands.registerCommand("scoreCoral", targetingSystem.setBranchLevel(ReefBranchLevel.L4).andThen(scoringSystem.scoreCoralAuto()));
    NamedCommands.registerCommand("scoreProcessor", scoringSystem.scoreAlgaeProcessorAuto());
    NamedCommands.registerCommand("loadCoral", loadingSystem.coralLoadAuto());
    NamedCommands.registerCommand("loadAlgae", loadingSystem.algaeLoadAuto());
    NamedCommands.registerCommand("LimbsDown", ( elevator.CoralHP()
                                                    .alongWith(coralArm.setCoralArmAngle(Setpoints.Arm.Coral.HP)
                                                    .until(elevator.aroundCoralHP()
                                                    .and(coralArm.aroundCoralHPAngle())))));         
    NamedCommands.registerCommand("LimbsUp", (elevator.CoralL4()
                                                  .alongWith(coralArm.setCoralArmAngle(Setpoints.Arm.Coral.L4)
                                                  .until(elevator.aroundCoralL4()
                                                  .and(coralArm.aroundCoralL4())))));
    NamedCommands.registerCommand("climber UP", climb.up());
    NamedCommands.registerCommand("climber DOWN", climb.down());
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

    // m_driverController.povUp().whileTrue(climb.climbUp());
    // m_driverController.povDown().whileTrue(climb.climbDown());
    
    m_driverController.leftBumper()
                      .whileTrue(Commands.run(() -> driveAngularVelocity.scaleTranslation(0.4))); // Slow mode
    m_driverController.leftBumper()
                      .whileFalse(Commands.run(() -> driveAngularVelocity.scaleTranslation(0.8)));//Fast mode

    m_driverController.povUp().whileTrue(climb.climb());
    m_driverController.povDown().whileTrue(climb.down());
    //m_driverController.y().onTrue(drivebase.rotateToHeading((Rotation2d.fromDegrees(90))).withTimeout(1));
    m_driverController.button(1).onTrue(Commands.print("Turn 90 Counter-Clockwise"));

    m_driverController.b().onTrue(targetingSystem.autoTargetCommand(drivebase::getPose)
                                  .andThen( Commands.runOnce(() ->
                                  drivebase.getSwerveDrive().field.getObject("target")))
                                  );

    //DRIVER CONTROLS ^
//--------------------------------------------------------------------------------------------------------------------------------------
    //OPERATOR CONTROLS - Launchpad v

    
    boolean launchpadTesting = true;
    if (launchpadTesting)
    {
    
    coralArm.coralLoadedTrigger().whileFalse( Commands.runOnce(()->launchpad.changeLED(0,8, new Color8Bit(ButtonColours.CoralUnloaded))))
                                 .whileTrue(Commands.runOnce(()->launchpad.changeLED(0,8, new Color8Bit(ButtonColours.CoralLoaded)))); 

    algaeArm.algaeLoadedTrigger().whileFalse( Commands.runOnce(()->launchpad.changeLED(0,7, new Color8Bit(ButtonColours.AlgaeUnloaded))))
                                 .whileTrue(Commands.runOnce(()->launchpad.changeLED(0,7, new Color8Bit(ButtonColours.AlgaeLoaded)))); 
    //Colours

    //Coral Level Select
    launchpad.getButton(0, 4).whileFalse( Commands.runOnce(()->launchpad.changeLED(0,4, new Color8Bit(ButtonColours.CoralLevelColour))))
                                 .whileTrue( Commands.runOnce(()->launchpad.changeLED(0,4, new Color8Bit(ButtonColours.IsPressed)))
                                 .alongWith(levelHighlighter1())); 

    launchpad.getButton(0, 3).whileFalse( Commands.runOnce(()->launchpad.changeLED(0,3, new Color8Bit(ButtonColours.CoralLevelColour))))
                                  .whileTrue( Commands.runOnce(()->launchpad.changeLED(0,3, new Color8Bit(ButtonColours.IsPressed)))
                                  .alongWith(levelHighlighter2()));  

    launchpad.getButton(0, 2).whileFalse( Commands.runOnce(()->launchpad.changeLED(0,2, new Color8Bit(ButtonColours.CoralLevelColour))))
                                  .whileTrue( Commands.runOnce(()->launchpad.changeLED(0,2, new Color8Bit(ButtonColours.IsPressed)))
                                  .alongWith(levelHighlighter3()));   
                                
    launchpad.getButton(0, 1).whileFalse( Commands.runOnce(()->launchpad.changeLED(0,1, new Color8Bit(ButtonColours.CoralLevelColour))))
                                  .whileTrue( Commands.runOnce(()->launchpad.changeLED(0,1, new Color8Bit(ButtonColours.IsPressed)))
                                  .alongWith(levelHighlighter4()));   
    //Score Coral                              
    launchpad.getButton(7, 8).whileFalse( Commands.runOnce(()->launchpad.changeLED(7,8, new Color8Bit(ButtonColours.ScoreCoral))))
                                  .whileTrue( Commands.runOnce(()->launchpad.changeLED(7,8, new Color8Bit(ButtonColours.IsPressed)))
                                  .alongWith(levelDeselect())); 

    //Algae
    launchpad.getButton(1, 2).whileFalse( Commands.runOnce(()->launchpad.changeLED(1,2, new Color8Bit(ButtonColours.AlgaeColour))))
                                  .whileTrue( Commands.runOnce(()->launchpad.changeLED(1,2, new Color8Bit(ButtonColours.IsPressed))));    
         
    launchpad.getButton(1, 3).whileFalse( Commands.runOnce(()->launchpad.changeLED(1,3, new Color8Bit(ButtonColours.AlgaeColour))))
                                  .whileTrue( Commands.runOnce(()->launchpad.changeLED(1,3, new Color8Bit(ButtonColours.IsPressed))));    
                                 
    launchpad.getButton(1, 0).whileFalse( Commands.runOnce(()->launchpad.changeLED(1,0, new Color8Bit(ButtonColours.AlgaeColour))))
                                  .whileTrue( Commands.runOnce(()->launchpad.changeLED(1,0, new Color8Bit(ButtonColours.IsPressed))));    
    
    launchpad.getButton(2, 0).whileFalse( Commands.runOnce(()->launchpad.changeLED(2,0, new Color8Bit(ButtonColours.AlgaeColour))))
                                  .whileTrue( Commands.runOnce(()->launchpad.changeLED(2,0, new Color8Bit(ButtonColours.IsPressed))));  
                                  
    launchpad.getButton(3, 0).whileFalse( Commands.runOnce(()->launchpad.changeLED(3,0, new Color8Bit(Color.kRed))))
                                  .whileTrue( Commands.runOnce(()->launchpad.changeLED(3,0, new Color8Bit(ButtonColours.IsPressed))));  
                                  
    launchpad.getButton(4, 0).whileFalse( Commands.runOnce(()->launchpad.changeLED(4,0, new Color8Bit(Color.kBlue))))
                                  .whileTrue( Commands.runOnce(()->launchpad.changeLED(4,0, new Color8Bit(ButtonColours.IsPressed))));  
    //HP
  
    launchpad.getButton(7, 1).whileFalse( Commands.runOnce(()->launchpad.changeLED(7,1, new Color8Bit(ButtonColours.HP))))
                                  .whileTrue( Commands.runOnce(()->launchpad.changeLED(7,1, new Color8Bit(ButtonColours.HPPressed))));

    launchpad.getButton(8, 1).whileFalse( Commands.runOnce(()->launchpad.changeLED(8,1, new Color8Bit(Color.kLimeGreen))))
                                  .whileTrue( Commands.runOnce(()->launchpad.changeLED(8,1, new Color8Bit(ButtonColours.IsPressed))));
                                  
    launchpad.getButton(8, 2).whileFalse( Commands.runOnce(()->launchpad.changeLED(8,2, new Color8Bit(Color.kMediumPurple))))
                                  .whileTrue( Commands.runOnce(()->launchpad.changeLED(8,2, new Color8Bit(ButtonColours.IsPressed))));

//-------------------------------------------------------------------------------------------------------------------------------------------------

      //L1 Score Coral
      launchpad.getButton(0, 4).whileTrue(targetingSystem.autoTargetCommand(drivebase::getPose)
                                                        .andThen( Commands.runOnce(() ->
                                                        drivebase.getSwerveDrive().field.getObject("target").setPose(targetingSystem.getCoralTargetPose())))
                                                        .andThen(targetingSystem.setBranchLevel(ReefBranchLevel.L1))
                                                        );
      //L2 Score Coral
      launchpad.getButton(0, 3).whileTrue(targetingSystem.autoTargetCommand(drivebase::getPose)
                                                        .andThen( Commands.runOnce(() ->
                                                        drivebase.getSwerveDrive().field.getObject("target").setPose(targetingSystem.getCoralTargetPose())))

                                                        .andThen(targetingSystem.setBranchLevel(ReefBranchLevel.L2))
                                                        );
      //L3 Score Coral
      launchpad.getButton(0, 2).whileTrue(targetingSystem.autoTargetCommand(drivebase::getPose)
                                                        .andThen( Commands.runOnce(() ->
                                                        drivebase.getSwerveDrive().field.getObject("target").setPose(targetingSystem.getCoralTargetPose())))

                                                        .andThen(targetingSystem.setBranchLevel(ReefBranchLevel.L3))
                                                        );
      //L4 Score Coral
      launchpad.getButton(0, 1).whileTrue(targetingSystem.autoTargetCommand(drivebase::getPose)
                                                        .andThen( Commands.runOnce(() ->
                                                        drivebase.getSwerveDrive().field.getObject("target").setPose(targetingSystem.getCoralTargetPose())))
                                                        .andThen(targetingSystem.setBranchLevel(ReefBranchLevel.L4))
                                                        );

      launchpad.getButton(3, 0).whileTrue(targetingSystem.setBranchSide(ReefBranchSide.LEFT));
      launchpad.getButton(4, 0).whileTrue(targetingSystem.setBranchSide(ReefBranchSide.RIGHT));

      //score coral
      launchpad.getButton(7, 8).whileTrue(scoringSystem.scoreCoral());

     

      //Algae Load L23
      launchpad.getButton(1, 2).whileTrue(targetingSystem.setBranchLevel(ReefBranchLevel.L2) 
                                                          .andThen( Commands.runOnce(() ->
                                                          drivebase.getSwerveDrive().field.getObject("target").setPose(targetingSystem.getAlgaeTargetPose())))
                                                         .andThen(targetingSystem.autoTargetCommand(drivebase::getPose))
                                                         .andThen(loadingSystem.algaeLoad()));

      //Algae Load L34
      launchpad.getButton(1, 3).whileTrue(targetingSystem.autoTargetCommand(drivebase::getPose)
                                                          .andThen( Commands.runOnce(() ->
                                                          drivebase.getSwerveDrive().field.getObject("target").setPose(targetingSystem.getAlgaeTargetPose())))
                                                         .andThen(targetingSystem.setBranchLevel(ReefBranchLevel.L3))
                                                         .andThen(loadingSystem.algaeLoad()));

      //Score Net
      launchpad.getButton(1, 0).whileTrue(scoringSystem.scoreAlgaeNet());

      //Score Processor
      launchpad.getButton(2, 0).whileTrue(algaeArm.PROCESSOR().repeatedly().alongWith(Commands.waitSeconds(0.3).andThen(algaeIntake.out())));

      // m_OperatorController1.button(19).onTrue(loadingSystem.coralLock());


      launchpad.getButton(7, 1).whileTrue(coralArm.setCoralArmAngle(Setpoints.Arm.Coral.HP));
      launchpad.getButton(4, 3).whileTrue(coralIntake.wristIntake());
      launchpad.changeLED(4, 3, new Color8Bit(Color.kRed));
      launchpad.getButton(5, 3).whileTrue(coralIntake.wristScore());
      launchpad.changeLED(5, 3, new Color8Bit(Color.kWhite));
      launchpad.getButton(6, 3).whileTrue(coralIntake.wristOuttake());
      launchpad.changeLED(6, 3, new Color8Bit(Color.kPurple));

      launchpad.getButton(4, 4).whileTrue(algaeIntake.in());
      launchpad.changeLED(4, 4, new Color8Bit(Color.kDarkGreen));
      launchpad.getButton(5, 4).whileTrue(algaeIntake.setAlgaeIntakeRoller(0));
      launchpad.changeLED(5, 4, new Color8Bit(Color.kWhite));
      launchpad.getButton(6, 4).whileTrue(algaeIntake.out());
      launchpad.changeLED(6, 4, new Color8Bit(Color.kLimeGreen));

      launchpad.getButton(4, 5).whileTrue(elevator.setPower(0.5).unless(elevator.atMax));
      launchpad.changeLED(4, 5, new Color8Bit(Color.kYellow));
      launchpad.getButton(5,5).whileTrue(elevator.hold());
      launchpad.changeLED(5,5, new Color8Bit(Color.kWhite));
      launchpad.getButton(6,5).whileTrue(elevator.setPower(-0.4).unless(elevator.atMin));
      launchpad.changeLED(6, 5, new Color8Bit(Color.kChocolate));

      launchpad.changeLED(6,8, new Color8Bit(Color.kGreen));
      launchpad.getButton(6,8).whileTrue(elevator.setElevatorHeight(0.3).repeatedly().alongWith(Commands.waitSeconds(0.3)
      .andThen(algaeArm.setAlgaeArmAngle(-20).repeatedly())));

      launchpad.changeLED(5,8, new Color8Bit(Color.kMediumPurple));
      launchpad.getButton(5,8).whileTrue(elevator.setElevatorHeight(0.3).repeatedly().alongWith(Commands.waitSeconds(0.3)
      .andThen(coralArm.setCoralArmAngle(-20).repeatedly())));




      launchpad.getButton(8, 2).whileTrue(coralArm.setCoralArmAngle(-60));
      launchpad.getButton(8,1).whileTrue(algaeArm.setAlgaeArmAngle(-60));

      launchpad.getButton(7, 0).whileTrue(coralArm.setCoralArmAngle(0));
      launchpad.changeLED(7,0, new Color8Bit(Color.kMediumPurple));
      launchpad.getButton(8,0).whileTrue(algaeArm.setAlgaeArmAngle(0));
      launchpad.changeLED(8,0, new Color8Bit(Color.kGreen));

      launchpad.getButton(8, 8).whileTrue(drivebase.printCurrentPose());

      //LAUNCH PAD ^
    }
      //--------------------------------------------------------------------------------------------------------------------
      //SIM BUTTON CONTROLS v

      //L1 Score Coral
      m_OperatorController1.a().whileTrue(targetingSystem.autoTargetCommand(drivebase::getPose)
                                                        .andThen( Commands.runOnce(() ->
                                                        drivebase.getSwerveDrive().field.getObject("target")))
                                                        .andThen(targetingSystem.setBranchLevel(ReefBranchLevel.L1))
                                                        );

      m_OperatorController1.b().whileTrue(targetingSystem.autoTargetCommand(drivebase::getPose)
                                                        .andThen( Commands.runOnce(() ->
                                                        drivebase.getSwerveDrive().field.getObject("target")))
                                                        .andThen(targetingSystem.setBranchLevel(ReefBranchLevel.L2))
                                                        );

      m_OperatorController1.x().whileTrue(targetingSystem.autoTargetCommand(drivebase::getPose)
                                                        .andThen( Commands.runOnce(() ->
                                                        drivebase.getSwerveDrive().field.getObject("target")))
                                                        .andThen(targetingSystem.setBranchLevel(ReefBranchLevel.L3))
                                                        );

      m_OperatorController1.y().whileTrue(targetingSystem.autoTargetCommand(drivebase::getPose)
                                                        .andThen( Commands.runOnce(() ->
                                                        drivebase.getSwerveDrive().field.getObject("target")))
                                                        .andThen(targetingSystem.setBranchLevel(ReefBranchLevel.L4))
                                                        );

      m_OperatorController1.rightTrigger().whileTrue(scoringSystem.scoreCoral());
      m_OperatorController1.rightBumper().whileTrue(algaeIntake.out().finallyDo(()->algaeIntake.setAlgaeIntakeRoller(0)));

      m_OperatorController1.povLeft().whileTrue(scoringSystem.scoreAlgaeNet());
      m_OperatorController1.povRight().whileTrue(algaeArm.PROCESSOR().alongWith(algaeIntake.out()));

      m_OperatorController1.povDown().whileTrue(targetingSystem.setBranchLevel(ReefBranchLevel.L2) 
                                      .andThen( Commands.runOnce(() ->
                                      drivebase.getSwerveDrive().field.getObject("target")))
                                    .andThen(targetingSystem.autoTargetCommand(drivebase::getPose))
                                    .andThen(loadingSystem.algaeLoad()));

      //Algae Load L34
      m_OperatorController1.povUp().whileTrue(targetingSystem.autoTargetCommand(drivebase::getPose)
                                    .andThen( Commands.runOnce(() ->
                                    drivebase.getSwerveDrive().field.getObject("target")))
                                  .andThen(targetingSystem.setBranchLevel(ReefBranchLevel.L3))
                                  .andThen(loadingSystem.algaeLoad()));

      m_OperatorController1.leftTrigger().whileTrue(coralArm.setCoralArmAngle(Setpoints.Arm.Coral.HP)
                                        .alongWith(coralIntake.wristIntake()));

      m_OperatorController1.leftBumper().whileTrue(algaeArm.setAlgaeArmAngle(-60));
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
    return drivebase.getAutonomousCommand("RightAuto1 Fast");
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
  public Command levelHighlighter1 (){
    return     Commands.runOnce(()->launchpad.changeLED(0,5, new Color8Bit(ButtonColours.notSelectedColour)))
    .alongWith(Commands.runOnce(()->launchpad.changeLED(0,6, new Color8Bit(ButtonColours.notSelectedColour))),
               Commands.runOnce(()->launchpad.changeLED(0,7, new Color8Bit(ButtonColours.notSelectedColour))),
               Commands.runOnce(()->launchpad.changeLED(0,8, new Color8Bit(ButtonColours.selectedColour))));
}

public Command levelHighlighter2 (){
    return     Commands.runOnce(()->launchpad.changeLED(0,5, new Color8Bit(ButtonColours.notSelectedColour)))
    .alongWith(Commands.runOnce(()->launchpad.changeLED(0,6, new Color8Bit(ButtonColours.notSelectedColour))),
               Commands.runOnce(()->launchpad.changeLED(0,7, new Color8Bit(ButtonColours.selectedColour))),
               Commands.runOnce(()->launchpad.changeLED(0,8, new Color8Bit(ButtonColours.notSelectedColour))));
}

public Command levelHighlighter3 (){
    return     Commands.runOnce(()->launchpad.changeLED(0,5, new Color8Bit(ButtonColours.notSelectedColour)))
    .alongWith(Commands.runOnce(()->launchpad.changeLED(0,6, new Color8Bit(ButtonColours.selectedColour))),
               Commands.runOnce(()->launchpad.changeLED(0,7, new Color8Bit(ButtonColours.notSelectedColour))),
               Commands.runOnce(()->launchpad.changeLED(0,8, new Color8Bit(ButtonColours.notSelectedColour))));
}

public Command levelHighlighter4 (){
    return     Commands.runOnce(()->launchpad.changeLED(0,5, new Color8Bit(ButtonColours.selectedColour)))
    .alongWith(Commands.runOnce(()->launchpad.changeLED(0,6, new Color8Bit(ButtonColours.notSelectedColour))),
               Commands.runOnce(()->launchpad.changeLED(0,7, new Color8Bit(ButtonColours.notSelectedColour))),
               Commands.runOnce(()->launchpad.changeLED(0,8, new Color8Bit(ButtonColours.notSelectedColour))));
}
public Command levelDeselect (){
  return     Commands.runOnce(()->launchpad.changeLED(0,5, new Color8Bit(ButtonColours.notSelectedColour)))
  .alongWith(Commands.runOnce(()->launchpad.changeLED(0,6, new Color8Bit(ButtonColours.notSelectedColour))),
             Commands.runOnce(()->launchpad.changeLED(0,7, new Color8Bit(ButtonColours.notSelectedColour))),
             Commands.runOnce(()->launchpad.changeLED(0,8, new Color8Bit(ButtonColours.notSelectedColour))));
}
}
