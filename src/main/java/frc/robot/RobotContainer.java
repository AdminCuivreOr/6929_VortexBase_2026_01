// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

// PathPlanner
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;

// WPILib - Math & Geometry
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;

// WPILib - Core
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotBase;

// WPILib - Dashboard
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

// WPILib - Command-based
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
// WPILib - Input
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import static edu.wpi.first.units.Units.RPM;

// Java
import java.io.File;

// SwerveLib
import swervelib.SwerveInputStream;
import swervelib.encoders.SwerveAbsoluteEncoder;

// Robot - Constants
import frc.robot.MainConstants;
import frc.robot.MainConstants.OperatorConstants;

// Robot - Swerve & Auto
import frc.robot.SwerveAndAuto.SwerveSubsystem;
import frc.robot.SwerveAndAuto.Autos;

// Robot - Intake
import frc.robot.Intake.IntakeSubsystem;
import frc.robot.Intake.IntakeCommand;

import frc.robot.Intake.BrasIntake.BrasIntakeSubsystem;
import frc.robot.Intake.BrasIntake.Brasintakedefault;
import frc.robot.Intake.BrasIntake.Brasintakedown;

// Robot - Shooter
import frc.robot.Shooter.ShooterSubsystem;
import frc.robot.Shooter.ShooterCommand;
import frc.robot.Shooter.ShooterCommandConstant;
import frc.robot.Shooter.Actuator.ActuatorSubsystem;
import frc.robot.Shooter.Actuator.MoveActuatorCommand;

// Robot - Tube
import frc.robot.Tube.TubeSubsystem;
import frc.robot.Tube.TubeCommand;

// Robot - Turret
import frc.robot.Turret.TurretSubsystem;
import frc.robot.Turret.AlignTurret;
import frc.robot.Turret.AlignTurretAuto;
// Robot - Climb
import frc.robot.Climb.ClimbSubsystem;
import frc.robot.Climb.ClimbingCommand;

// Robot - Exemple
import frc.robot.Autre.ExampleCommand;
import frc.robot.Autre.ExampleSubsystem;
/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...

  // Moteurs en minuscule 
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  private final SwerveSubsystem drivebase = new SwerveSubsystem();
  private final TurretSubsystem turret = new TurretSubsystem();   
  private final ShooterSubsystem m_Shooter = new ShooterSubsystem();
  private final TubeSubsystem tube = new TubeSubsystem();
  private final IntakeSubsystem intake = new IntakeSubsystem();
  private final ClimbSubsystem m_grimpeur = new ClimbSubsystem();
  private final BrasIntakeSubsystem m_Brasintake = new BrasIntakeSubsystem();
  private final ActuatorSubsystem actuator = new ActuatorSubsystem();

  //path planner command en majuscule 💀🥀😔⚰️⚰️🍂‼️
 private final Command shooterCommand3 = new ShooterCommand(m_Shooter, drivebase).withTimeout(3.0);
 private final Command shooterCommand = new ShooterCommand(m_Shooter, drivebase).withTimeout(5.0);
 private final Command AlignTurret = new AlignTurret(turret).withTimeout(2.0);
 private final Command IntakeCommand = new IntakeCommand(intake, -0.75).withTimeout(20.0);
 private final Command BrasIntakeDown = new Brasintakedown(m_Brasintake);
 private final Command TubeCommand = new TubeCommand(tube, 0.75, -0.75).withTimeout(5.0);


  // Establish a Sendable Chooser that will be able to be sent to the SmartDashboard, allowing selection of desired auto
  private final SendableChooser<Command> autoChooser;

  private boolean m_fieldOriented = true;
  private double speedMult = 1.0;
  private double CurrentRPM = -4000.0;

  private boolean brasOUT = false ;
  // Replace with CommandPS4Controller or CommandJoystick if needed

  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);
  private final Joystick m_copilote = new Joystick(1);
  

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    //NamedCommand for path planner
   NamedCommands.registerCommand("Shooter", shooterCommand);
   NamedCommands.registerCommand("Shooter3", shooterCommand3);
   NamedCommands.registerCommand("AlignTurret", AlignTurret);
   NamedCommands.registerCommand("BrasIntakeDown", BrasIntakeDown);
   NamedCommands.registerCommand("IntakeIn", IntakeCommand);
   NamedCommands.registerCommand("Tube", TubeCommand);
    // Configure the trigger bindings
    configureBindings();
    drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity);
    SmartDashboard.putBoolean("Field Oriented", m_fieldOriented);

    //Have the autoChooser pull in all PathPlanner autos as options
    autoChooser = AutoBuilder.buildAutoChooser();
    //Set the default auto (do nothing) 
    autoChooser.setDefaultOption("Do Nothing", Commands.none());
    //Put the autoChooser on the SmartDashboard
    SmartDashboard.putData("Auto Chooser", autoChooser);

     new RunCommand(() -> {                         
        double value = (drivebase.getTagCount() > 0) ? 0.2 : 0.0; // rumble si oui : si non 
        m_driverController.setRumble(GenericHID.RumbleType.kBothRumble, value);
    }).ignoringDisable(true).schedule();
}
  

   SwerveInputStream driveAngularVelocity = SwerveInputStream.of(drivebase.getSwerveDrive(),
                                                                () -> m_driverController.getLeftY() * -1 * speedMult, 
                                                                () -> m_driverController.getLeftX() * -1 * speedMult ) 
                                                            .withControllerRotationAxis(m_driverController::getRightX)
                                                            .deadband(OperatorConstants.DEADBAND)
                                                            .scaleTranslation(0.8)
                                                            .allianceRelativeControl(() -> m_fieldOriented)
                                                            .robotRelative(() -> !m_fieldOriented);
                                                            


  SwerveInputStream driveDirectAngle = driveAngularVelocity.copy().withControllerHeadingAxis(m_driverController::getRightX,
                                                                                             m_driverController::getRightY)
                                                            .headingWhile(true);


  SwerveInputStream driveRobotOriented = driveAngularVelocity.copy().robotRelative(true)
                                                                .allianceRelativeControl(false);

   Command driveFieldOrientedDirectAngle      = drivebase.driveFieldOriented(driveDirectAngle);
   Command driveFieldOrientedAnglularVelocity = drivebase.driveFieldOriented(driveAngularVelocity);
   Command driveRobotOrientedAngularVelocity = drivebase.driveFieldOriented(driveRobotOriented);
   

   //Command driveTeleop = new ConditionalCommand(
     //   driveFieldOrientedAnglularVelocity,
      //  driveRobotOrientedAngularVelocity,
      //  () -> m_fieldOriented
    //);
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

    //Noms de mécasnismes activés en Majuscule

    POVButton Turret = new POVButton(m_copilote, 0); // Pov haut
    JoystickButton ShooterConstant = new JoystickButton(m_copilote, 3); // x
    //POVButton ShooterConstant = new POVButton(m_copilote, 180);
    JoystickButton Tube = new JoystickButton(m_copilote, 1); // a
    JoystickButton IntakeIn = new JoystickButton(m_copilote, 5); //bumber gauche
    JoystickButton IntakeOut = new JoystickButton(m_copilote, 6); // bumber droit

    JoystickButton BrasIntakeOut = new JoystickButton(m_copilote, 4); // y
    JoystickButton BrasIntakeIn = new JoystickButton(m_copilote, 2); // b
    POVButton Actuator50 = new POVButton(m_copilote, 90); // droite?
    JoystickButton RPMplus = new JoystickButton(m_copilote, 9); //bouton temporaire
    JoystickButton RPMmoins = new JoystickButton(m_copilote, 10);
    
    //JoystickButton ActuatorExtend = new JoystickButton(m_copilote, 5); // bumber gauche
    //JoystickButton ActuatorRetract = new JoystickButton(m_copilote, 6); // bumber droit
    //POVButton ActuatorPos50 = new POVButton(m_copilote, 90); // Droite

    //JoystickButton Brasintakedefault = new JoystickButton(m_copilote, 9);
    //JoystickButton Brasintakedown = new JoystickButton(m_copilote, 10);

  

    new Trigger(m_exampleSubsystem::exampleCondition)
        .onTrue(new ExampleCommand(m_exampleSubsystem));

    //new Trigger().whileTrue(driveRobotOriented); 

    // Schedule `exampleMethodCommand` when the Xbox cont roller's B button is pressed,
    // cancelling on release.<
      m_driverController.back().onTrue(
    new InstantCommand(() -> drivebase.zeroHeading(), drivebase));
   
   Turret.whileTrue(new AlignTurret(turret));
  // Shooter.whileTrue(new ShooterCommand(m_Shooter, drivebase));//Shooter active actuator 50 %
   Actuator50.whileTrue(actuator.setPositionPercentCommand(50)); // étendre à 50 %
   Actuator50.whileFalse(actuator.setPositionPercentCommand(0)); // rétrater lorsque relâché

   Tube.whileTrue(new TubeCommand(tube, 0.90, -0.90));
   Tube.whileTrue(new IntakeCommand(intake, -0.75));
   IntakeIn.whileTrue(new IntakeCommand(intake, -0.75));
   IntakeOut.whileTrue(new IntakeCommand(intake, 0.75));

   ShooterConstant.whileTrue(new ShooterCommandConstant(m_Shooter));

   //temporaire :

   RPMmoins.onTrue(Commands.runOnce(()-> {
    CurrentRPM = CurrentRPM + 100;
    SmartDashboard.putNumber("VariableRPMShooter", CurrentRPM);
   }));

   RPMplus.onTrue(Commands.runOnce(() -> {
    CurrentRPM = CurrentRPM - 100;
    SmartDashboard.putNumber("VariableRPMShooter", CurrentRPM);
   }));
    //o
  /* 
   if (m_copilote.getRawButtonPressed(4)){ // le bouton 4 c'est Y
    brasOUT = !brasOUT;
    if (brasOUT) {
        System.out.println("down");
        new Brasintakedown(m_Brasintake).schedule();
    } else {
        System.out.println("default");
        new Brasintakedefault(m_Brasintake).schedule();
    }
   } **/

    BrasIntakeOut.onTrue(new Brasintakedown(m_Brasintake));
    BrasIntakeIn.onTrue(new Brasintakedefault(m_Brasintake));
    
    m_driverController.back().onTrue(
            new InstantCommand(() -> drivebase.zeroHeading(), drivebase));

    m_driverController.start().onTrue( // Démarrer FO (FC)
    Commands.runOnce(() -> {
      m_fieldOriented = !m_fieldOriented;
      SmartDashboard.putBoolean("Field Oriented", m_fieldOriented);

    })
    );

   


    m_driverController.leftBumper().onTrue(Commands.runOnce(() -> {
      speedMult = 0.2; // Si bumber gauche maintenu : Vitesse lente
    })).onFalse(Commands.runOnce(() -> {
      speedMult = 2.0; // Si bumber gauche relaché : Vitesse maximale
    }));
       
    m_driverController.b().whileTrue(m_exampleSubsystem.exampleMethodCommand());
  }
 

  /**
   * Get the path follower with events.
   *
   * @param pathName PathPlanner path name.
   * @return {@link AutoBuilder#followPath(PathPlannerPath)} path command.
   */
  
  public Command getAutonomousCommand()
  {
    // Pass in the selected auto from the SmartDashboard as our desired autnomous commmand 
    return autoChooser.getSelected();
  }
}

