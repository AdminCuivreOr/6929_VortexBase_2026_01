// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.LimelightHelpers;

import java.util.Optional;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class AlignTurret extends Command {
  private final TurretSubsystem m_turret;
  private final SwerveSubsystem m_swerve;


  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public AlignTurret(TurretSubsystem turret, SwerveSubsystem swerve) {
    m_turret = turret;
    m_swerve = swerve;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(turret);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    var pos = m_swerve.getPose();
    Translation2d target = new Translation2d(0, 0);;

    Optional<Alliance> ally = DriverStation.getAlliance();

    if (ally.isPresent()) {
       if (ally.get() == Alliance.Red) {
          target = new Translation2d(11.916, 4.035);
         }
       if (ally.get() == Alliance.Blue) {
          target = new Translation2d(4.63, 4.035);
       }
      }
    else {
          System.out.println("aucune couleur d'alliance");
    }

    Translation2d robotTarget = target.minus(pos.getTranslation()); // mauvais calcul ?
    Rotation2d angle = robotTarget.getAngle().minus(pos.getRotation()); // ou lui.

    m_turret.moveToAngle(-angle.getDegrees());
    


    SmartDashboard.putNumber("Turret/AngleRAWRobotTarget", angle.getDegrees());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_turret.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
