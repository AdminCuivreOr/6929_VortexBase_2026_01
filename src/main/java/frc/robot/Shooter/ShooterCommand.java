package frc.robot.Shooter;

import edu.wpi.first.wpilibj2.command.Command;

public class ShooterCommand extends Command {

    private final ShooterSubsystem shooter;
    private final double rpm;

    public ShooterCommand(ShooterSubsystem shooter, double rpm) {
        this.shooter = shooter;
        this.rpm = rpm;

        addRequirements(shooter);
    }

    @Override
    public void initialize() {
        // On commence à mettre le shooter à la vitesse désirée
        shooter.setRPM(rpm);
    }

    @Override
    public void execute() {
        // On continue à maintenir la vitesse cible
        shooter.setRPM(rpm);
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