package frc.robot.SwerveAndAuto;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.LimelightHelpers;

public class AutoAimSwerve extends Command {

    private final SwerveSubsystem m_swerve;

    private final DoubleSupplier xSupplier;
    private final DoubleSupplier ySupplier;

    public AutoAimSwerve(
        SwerveSubsystem swerve,
        DoubleSupplier xSupplier,
        DoubleSupplier ySupplier
    ) {

        this.m_swerve = swerve;

        this.xSupplier = xSupplier;
        this.ySupplier = ySupplier;

        addRequirements(swerve);
    }

    @Override
    public void execute() {

        // Translation contrôlée par le pilote
        double xSpeed =
            MathUtil.applyDeadband(
                -xSupplier.getAsDouble(),
                0.05
            ) * 4.0;

        double ySpeed =
            MathUtil.applyDeadband(
                -ySupplier.getAsDouble(),
                0.05
            ) * 4.0;

        double output = 0;

        boolean hasTarget =
            LimelightHelpers.getTV("limelight");

        if (hasTarget) {

            // Erreur horizontale de la cible
            double tx =
                LimelightHelpers.getTX("limelight");

            // Contrôle proportionnel
            output = tx * 0.08;

            // Zone morte pour éviter les oscillations
            if (Math.abs(tx) < 1.0) {
                output = 0;
            }

            // Limite vitesse de rotation
            output = MathUtil.clamp(
                output,
                -3.0,
                3.0
            );

            SmartDashboard.putNumber(
                "AutoAim TX",
                tx
            );

            SmartDashboard.putNumber(
                "AutoAim output",
                output
            );

        } else {

            SmartDashboard.putString(
                "AutoAim",
                "No Target"
            );
        }

        m_swerve.drive(
            xSpeed,
            ySpeed,
            output,
            true
        );
    }

    @Override
    public void end(boolean interrupted) {

        m_swerve.drive(
            0,
            0,
            0,
            true
        );
    }

    @Override
    public boolean isFinished() {

        return false;
    }
}