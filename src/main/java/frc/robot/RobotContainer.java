// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.AlgaeArmSubsystem;
import frc.robot.subsystems.AlgaeIntakeSubsystem;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.CoralArmSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.FloorIntakeSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import swervelib.SwerveInputStream;

import static edu.wpi.first.units.Units.Meter;

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

  private final ElevatorSubsystem     elevator           = new ElevatorSubsystem();
  private final CoralArmSubsystem arm                = new CoralArmSubsystem();
  private final ClimberSubsystem climb = new ClimberSubsystem();
  private final AlgaeIntakeSubsystem algaeIntake = new AlgaeIntakeSubsystem();
  private final AlgaeArmSubsystem algaeArm = new AlgaeArmSubsystem();
  private final FloorIntakeSubsystem floorIntake = new FloorIntakeSubsystem();


  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    DriverStation.silenceJoystickConnectionWarning(true);
    elevator.setDefaultCommand(elevator.setGoal(0));
    arm.setDefaultCommand(arm.setGoal(0));
    climb.setDefaultCommand(climb.climbUp());
    algaeIntake.setDefaultCommand(algaeIntake.setAlgaeIntakeAngle(0));
    algaeArm.setDefaultCommand(algaeArm.setGoal(0));
    floorIntake.setDefaultCommand(floorIntake.setCoralIntakeAngle(0));
    
    configureBindings();
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
  SwerveInputStream driveDirectAngleSim     = driveAngularVelocitySim.copy()
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
    
        m_driverController.button(17).whileTrue(LazyLazyDrive());

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

        m_driverController.button(6).whileTrue(drivebase.driveToPose(new Pose2d(new Translation2d
                                                                                (Meter.of(5),
                                                                                Meter.of(3)),
                                                                        Rotation2d.fromDegrees(-50))));

         m_driverController.button(5).whileTrue(drivebase.driveToPose(new Pose2d(new Translation2d
                                                                        (Meter.of(6.1),
                                                                        Meter.of(4)),
                                                                Rotation2d.fromDegrees(125))));

        m_driverController.button(4).whileTrue(drivebase.driveToPose(new Pose2d(new Translation2d
                                                                                (Meter.of(5.2),
                                                                                Meter.of(5.2)),
                                                                        Rotation2d.fromDegrees(-120))));


        m_driverController.button(3).whileTrue(drivebase.driveToPose(new Pose2d(new Translation2d
                                                                                (Meter.of(3.3),
                                                                                Meter.of(5.3)),
                                                                        Rotation2d.fromDegrees(-50))));
        //Processor
        m_driverController.button(2).whileTrue(drivebase.driveToPose(new Pose2d(new Translation2d
                                                                                (Meter.of(11.5),
                                                                                Meter.of(7.5)),
                                                                        Rotation2d.fromDegrees(90))));  
        //Human Playerstation                                                                                
        m_driverController.button(1).whileTrue(goToHumanPlayer1());  
                                                                                
            // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
            // cancelling on release.

            
            m_driverController.button(11).whileTrue(elevator.setGoal(3));
            m_driverController.button(12).whileTrue(elevator.setGoal(6));
            m_driverController.button(13).whileTrue(elevator.setGoal(9));
            m_driverController.button(14).whileTrue(arm.setGoal(45));
            m_driverController.button(15).whileTrue(arm.setGoal(90));
            m_driverController.button(16).whileTrue(setElevArm(10, 70));
            elevator.atHeight(5, 0.1).whileTrue(Commands.print("I AM ALIVE, YAAA HAAAAA"));
        
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

  public Command goToHumanPlayer1(){
    return drivebase.driveToPose(
    new Pose2d(new Translation2d
    (Meter.of(1),
     Meter.of(7)),
    Rotation2d.fromDegrees(130)));
  }

   public ParallelCommandGroup setElevArm (double goal, double degree){
  return  new ParallelCommandGroup(elevator.setGoal(goal), arm.setGoal(degree));
 }
   public ParallelCommandGroup LazyLazyDrive(){
    return new ParallelCommandGroup(   
      drivebase.driveToPose(
      new Pose2d(new Translation2d
      (Meter.of(1),
      Meter.of(7)),
Rotation2d.fromDegrees(130)))
    .andThen(
      drivebase.driveToPose(//Coral side 1
      new Pose2d(new Translation2d
      (Meter.of(3.4),
      Meter.of(5.1)),
      Rotation2d.fromDegrees(-50)))
    ).andThen(
      drivebase.driveToPose(//Human Player station
      new Pose2d(new Translation2d
      (Meter.of(1),
       Meter.of(7)),
      Rotation2d.fromDegrees(130)))
    ).andThen(
      drivebase.driveToPose(//Set point
      new Pose2d(new Translation2d
      (Meter.of(4.3),
      Meter.of(6.3)),
      Rotation2d.fromDegrees(-50)))
    ).andThen(
      drivebase.driveToPose( //Coral 2
      new Pose2d(new Translation2d
     (Meter.of(5.2),
     Meter.of(5.2)),
     Rotation2d.fromDegrees(-120)))
     )
    );
   }
}
