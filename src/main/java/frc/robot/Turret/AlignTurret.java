package frc.robot.Turret;

import frc.robot.LimelightHelpers;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class AlignTurret extends Command {

    private final TurretSubsystem m_turret;

    public AlignTurret(TurretSubsystem turret) {

        m_turret = turret;

        addRequirements(turret);
    }

    @Override
    public void execute() {

        boolean hasTarget = LimelightHelpers.getTV("limelight");

        if (hasTarget) {

            // erreur horizontale (en degrés)
            double tx = LimelightHelpers.getTX("limelight") * -0.237; //0.235 si augmente trop loin
            double angle = m_turret.getAngle();

            // contrôle proportionnel simple
            double output = (tx-angle) * 0.08;

            // limite vitesse moteur
            output = Math.max(-0.3, Math.min(0.3, output));

            m_turret.setMotor(output);

            SmartDashboard.putNumber("Turret/tx", tx-angle);
            SmartDashboard.putNumber("Turret/output", output);

        } else {

            m_turret.stop();
        }       
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