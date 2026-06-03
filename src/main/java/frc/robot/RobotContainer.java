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

// Robot - Tube
import frc.robot.Tube.TubeSubsystem;
import frc.robot.Tube.TubeCommand;

// Robot - Turret
import frc.robot.Turret.TurretSubsystem;
import frc.robot.Turret.AlignTurret;
import frc.robot.Turret.AlignTurretAuto;

// Robot - Exemple
import frc.robot.Autre.ExampleCommand;
import frc.robot.Autre.ExampleSubsystem;

/**
 * Classe principale qui relie :
 * - les subsystems
 * - les commandes
 * - les boutons de la manette
 */
public class RobotContainer {

  // ===================== SUBSYSTEMS =====================
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  private final SwerveSubsystem drivebase = new SwerveSubsystem();
  private final TurretSubsystem turret = new TurretSubsystem();   
  private final ShooterSubsystem m_Shooter = new ShooterSubsystem();
  private final TubeSubsystem tube = new TubeSubsystem();
  private final IntakeSubsystem intake = new IntakeSubsystem();
  private final BrasIntakeSubsystem m_Brasintake = new BrasIntakeSubsystem();

  // ===================== COMMANDES PRÉ-CRÉÉES =====================
  //path planner command en majuscule 💀🥀😔⚰️⚰️🍂‼️
  private final Command shooterCommand3 = new ShooterCommand(m_Shooter, drivebase).withTimeout(3.0);
  private final Command shooterCommand = new ShooterCommand(m_Shooter, drivebase).withTimeout(5.0);
  private final Command AlignTurret = new AlignTurret(turret, drivebase).withTimeout(2.0);
  private final Command IntakeCommand = new IntakeCommand(intake, -0.75).withTimeout(20.0);
  private final Command BrasIntakeDown = new Brasintakedown(m_Brasintake);
  private final Command TubeCommand = new TubeCommand(tube, 0.75, -0.75).withTimeout(5.0);

  // Choix autonome PathPlanner
  private final SendableChooser<Command> autoChooser;

  private boolean m_fieldOriented = true;
  private double speedMult = 1.0;
  private double CurrentRPM = -4000.0;

  // Manette pilote
  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);

  // Manette copilote
  private final Joystick m_copilote = new Joystick(1);

  /**
   * Constructeur : initialise tout le robot
   */
  public RobotContainer() {

    // Enregistrement des commandes PathPlanner
    NamedCommands.registerCommand("Shooter", shooterCommand);
    NamedCommands.registerCommand("Shooter3", shooterCommand3);
    NamedCommands.registerCommand("AlignTurret", AlignTurret);
    NamedCommands.registerCommand("BrasIntakeDown", BrasIntakeDown);
    NamedCommands.registerCommand("IntakeIn", IntakeCommand);
    NamedCommands.registerCommand("Tube", TubeCommand);

    configureBindings();

    // commande par défaut du swerve
    drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity);
    SmartDashboard.putBoolean("Field Oriented", m_fieldOriented);

    // création du chooser auto
    autoChooser = AutoBuilder.buildAutoChooser();
    autoChooser.setDefaultOption("Do Nothing", Commands.none());
    SmartDashboard.putData("Auto Chooser", autoChooser);

    // rumble si vision détecte des tags
    new RunCommand(() -> {                         
        double value = (drivebase.getTagCount() > 0) ? 0.2 : 0.0;
        m_driverController.setRumble(GenericHID.RumbleType.kBothRumble, value);
    }).ignoringDisable(true).schedule();
  }

  // ===================== SYSTÈME DE CONDUITE =====================

  SwerveInputStream driveAngularVelocity = SwerveInputStream.of(
      drivebase.getSwerveDrive(),
      () -> m_driverController.getLeftY() * -1 * speedMult,
      () -> m_driverController.getLeftX() * -1 * speedMult)
      .withControllerRotationAxis(m_driverController::getRightX)
      .deadband(OperatorConstants.DEADBAND)
      .scaleTranslation(0.8)
      .allianceRelativeControl(() -> m_fieldOriented)
      .robotRelative(() -> !m_fieldOriented);

  SwerveInputStream driveDirectAngle = driveAngularVelocity.copy()
      .withControllerHeadingAxis(
          m_driverController::getRightX,
          m_driverController::getRightY)
      .headingWhile(true);

  SwerveInputStream driveRobotOriented = driveAngularVelocity.copy()
      .robotRelative(true)
      .allianceRelativeControl(false);

  Command driveFieldOrientedDirectAngle = drivebase.driveFieldOriented(driveDirectAngle);
  Command driveFieldOrientedAnglularVelocity = drivebase.driveFieldOriented(driveAngularVelocity);
  Command driveRobotOrientedAngularVelocity = drivebase.driveFieldOriented(driveRobotOriented);

  // ===================== BOUTONS =====================
  private void configureBindings() {

    // boutons copilote
    POVButton Turret = new POVButton(m_copilote, 0);
    JoystickButton Shooter = new JoystickButton(m_copilote, 3);
    POVButton ShooterConstant = new POVButton(m_copilote, 180);

    JoystickButton Tube = new JoystickButton(m_copilote, 1);
    JoystickButton IntakeIn = new JoystickButton(m_copilote, 5);
    JoystickButton IntakeOut = new JoystickButton(m_copilote, 6);

    JoystickButton BrasIntakeOut = new JoystickButton(m_copilote, 4);
    JoystickButton BrasIntakeIn = new JoystickButton(m_copilote, 2);

    JoystickButton RPMplus = new JoystickButton(m_copilote, 9);
    JoystickButton RPMmoins = new JoystickButton(m_copilote, 10);

    // test subsystem
    new Trigger(m_exampleSubsystem::exampleCondition)
        .onTrue(new ExampleCommand(m_exampleSubsystem));

    // reset gyro
    m_driverController.back().onTrue(
        new InstantCommand(() -> drivebase.zeroHeading(), drivebase));

    // turret align
    Turret.whileTrue(new AlignTurret(turret, drivebase));

    // shooter normal
    Shooter.whileTrue(new ShooterCommand(m_Shooter, drivebase));

    // tube + intake combiné
    Tube.whileTrue(new TubeCommand(tube, 0.90, -0.90));
    Tube.whileTrue(new IntakeCommand(intake, -0.75));

    IntakeIn.whileTrue(new IntakeCommand(intake, -0.75));
    IntakeOut.whileTrue(new IntakeCommand(intake, 0.75));

    ShooterConstant.whileTrue(new ShooterCommandConstant(m_Shooter));

    // modification RPM manuel
    RPMmoins.onTrue(Commands.runOnce(() -> {
      CurrentRPM += 100;
      SmartDashboard.putNumber("VariableRPMShooter", CurrentRPM);
    }));

    RPMplus.onTrue(Commands.runOnce(() -> {
      CurrentRPM -= 100;
      SmartDashboard.putNumber("VariableRPMShooter", CurrentRPM);
    }));

    // bras intake
    BrasIntakeOut.onTrue(new Brasintakedown(m_Brasintake));
    BrasIntakeIn.onTrue(new Brasintakedefault(m_Brasintake));

    // toggle field oriented
    m_driverController.start().onTrue(
        Commands.runOnce(() -> {
          m_fieldOriented = !m_fieldOriented;
          SmartDashboard.putBoolean("Field Oriented", m_fieldOriented);
        })
    );

    // speed mode
    m_driverController.leftBumper()
        .onTrue(Commands.runOnce(() -> speedMult = 0.2))
        .onFalse(Commands.runOnce(() -> speedMult = 2.0));

    // test subsystem exemple
    m_driverController.b().whileTrue(m_exampleSubsystem.exampleMethodCommand());
  }

  /**
   * Retourne l’autonome sélectionné dans PathPlanner
   */
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}


//Michael est définitivement passé par là !