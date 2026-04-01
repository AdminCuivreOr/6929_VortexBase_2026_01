package frc.robot.Shooter;

import java.util.Optional;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.SwerveAndAuto.SwerveSubsystem;

public class ShooterCommand extends Command {

    private final ShooterSubsystem shooter;
    private final SwerveSubsystem swerve;

    public ShooterCommand(ShooterSubsystem shooter, SwerveSubsystem swerve) {
        this.shooter = shooter;
        this.swerve = swerve;

        addRequirements(shooter);
    }

    @Override
    public void execute() {
        var pos = swerve.getPose();
        double rpm;

        Optional<Alliance> ally = DriverStation.getAlliance();
        if (ally.isPresent()) {
            Translation2d target;

            if (ally.get() == Alliance.Red) {
                target = new Translation2d(11.916, 4.035);
            } else {
                target = new Translation2d(4.63, 4.035);
            }

            // calcul de la distance
            Translation2d robotToTarget = target.minus(pos.getTranslation());
            double distance = robotToTarget.getNorm();

            // clamp distance pour éviter des valeurs bizarres
            distance = Math.max(1.0, Math.min(5.5, distance));

            // calcul RPM via la courbe
            rpm = -getRPMFromDistance(distance);

            SmartDashboard.putNumber("Distance", distance);
        } else {
            // fallback si alliance inconnue
            rpm = -3500;
        }

        // clamp RPM négatif sécurisé
        rpm = Math.min(0, rpm);
        rpm = Math.max(-6000, rpm);

        // appliquer au shooter
        shooter.setRPM(rpm);
        SmartDashboard.putNumber("ShooterRPM", rpm);
    }

    private double getRPMFromDistance(double distance) {
        // formule quadratique
        return (-18.129 * distance * distance + 536.45 * distance + 1926.3)*1.00;
    }

    @Override
    public void end(boolean interrupted) {
        shooter.stop();
    }

    @Override
    public boolean isFinished() {
        return false; // tourne tant que le bouton est maintenu
    }
}