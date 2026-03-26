package frc.robot.Turret;

import frc.robot.LimelightHelpers;
import frc.robot.SwerveAndAuto.SwerveSubsystem;

import java.util.Optional;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class AlignTurret extends Command {

    private final TurretSubsystem m_turret;
    private final SwerveSubsystem m_swerve;

    public AlignTurret(TurretSubsystem turret, SwerveSubsystem swerve) {
        m_turret = turret;
        m_swerve = swerve;
       

        addRequirements(turret);
    }

    @Override
    public void execute() {
        var pos = m_swerve.getPose();
    

        Translation2d target = new Translation2d(0,0);

        Optional<Alliance> ally = DriverStation.getAlliance();
        if (ally.isPresent()) {
            if (ally.get() == Alliance.Red) {
                target = new Translation2d(11.916, 4.035);
            } else {
                target = new Translation2d(4.63, 4.035);
            }
        }
        else {
            System.out.println("aucune couleur d'alliance");
        }
        

Translation2d robotToTarget = target.minus(pos.getTranslation());
Rotation2d desiredAngle = robotToTarget.getAngle();
 
Rotation2d turretAngle = desiredAngle.minus(pos.getRotation());
 
var ll = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight");

       
        if (m_swerve.tagCountLL > 0) {
            m_turret.moveToAngle(turretAngle.getDegrees());
        } else {
            m_turret.moveToAngle(0);
        }

        SmartDashboard.putNumber("Turret/AngleRAWRobotTarget", turretAngle.getDegrees());
    }

    @Override
    public void end(boolean interrupted) {
        m_turret.stop();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}