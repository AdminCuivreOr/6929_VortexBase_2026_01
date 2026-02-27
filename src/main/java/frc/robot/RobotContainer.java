// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Autre.ExampleCommand;
import frc.robot.Autre.ExampleSubsystem;
import frc.robot.MainConstants.OperatorConstants;
import frc.robot.Intake.IntakeCommand;
import frc.robot.Intake.IntakeSubsystem;
import frc.robot.Shooter.ShooterCommand;
import frc.robot.Shooter.ShooterSubsystem;
import frc.robot.SwerveAndAuto.Autos;
import frc.robot.SwerveAndAuto.SwerveSubsystem;
import frc.robot.Tube.TubeCommand;
import frc.robot.Tube.TubeSubsystem;
import frc.robot.Turret.AlignTurret;
import frc.robot.Turret.TurretSubsystem;
import swervelib.SwerveInputStream;
import swervelib.encoders.SwerveAbsoluteEncoder;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.io.File;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  private final SwerveSubsystem drivebase = new SwerveSubsystem();
  private final TurretSubsystem turret = new TurretSubsystem();  //en minuscule 
  private final ShooterSubsystem m_Shooter = new ShooterSubsystem();
  private final TubeSubsystem tube = new TubeSubsystem();
  private final IntakeSubsystem intake = new IntakeSubsystem();

  //private final ActuatorSubsystem actuator = new ActuatorSubsystem();



  private boolean m_fieldOriented = true;
  
  // Replace with CommandPS4Controller or CommandJoystick if needed

  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);
  private final Joystick m_copilote = new Joystick(1);
      

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
    drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity);
    SmartDashboard.putBoolean("Field Oriented", m_fieldOriented);
  }

   SwerveInputStream driveAngularVelocity = SwerveInputStream.of(drivebase.getSwerveDrive(),
                                                                () -> m_driverController.getLeftY() * -1, //Cette valeur vaut habituellement -1
                                                                () -> m_driverController.getLeftX() * -1) //Cette valeur vaut habituellement -1
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

    JoystickButton Turret = new JoystickButton(m_copilote, 1); //a //en Majuscule
    JoystickButton Shooter = new JoystickButton(m_copilote, 2);//b
    JoystickButton Tube = new JoystickButton(m_copilote, 3);//x
    JoystickButton Intake = new JoystickButton(m_copilote, 4);//y

  //JoystickButton ActuatorExtend = new JoystickButton(m_copilote, 5); // bouton 7
  //JoystickButton ActuatorRetract = new JoystickButton(m_copilote, 6); // bouton 8

  

    new Trigger(m_exampleSubsystem::exampleCondition)
        .onTrue(new ExampleCommand(m_exampleSubsystem));

    //new Trigger().whileTrue(driveRobotOriented); 

    // Schedule `exampleMethodCommand` when the Xbox cont roller's B button is pressed,
    // cancelling on release.<
   
   Turret.whileTrue(new AlignTurret(turret, drivebase));
   Shooter.whileTrue(new ShooterCommand(m_Shooter, -0.55) );
   Tube.whileTrue(new TubeCommand(tube, 0.25, 0.55));
   Intake.whileTrue(new IntakeCommand(intake, 0.5));

  // ActuatorExtend.whileTrue(new MoveActuatorCommand(actuator, true));  // étendre
  // ActuatorRetract.whileTrue(new MoveActuatorCommand(actuator, false)); // rétracter


    
    m_driverController.start().onTrue(
    Commands.runOnce(() -> {
      m_fieldOriented = !m_fieldOriented;
      SmartDashboard.putBoolean("Field Oriented", m_fieldOriented);




    })
);


    m_driverController.b().whileTrue(m_exampleSubsystem.exampleMethodCommand());
  }




  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return Autos.exampleAuto(m_exampleSubsystem);
  }
}
