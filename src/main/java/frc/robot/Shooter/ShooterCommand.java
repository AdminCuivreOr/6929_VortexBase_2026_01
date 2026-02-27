package frc.robot.Shooter;
import edu.wpi.first.wpilibj2.command.Command;


public class ShooterCommand extends Command {

    private final ShooterSubsystem shooter;
    private final double speed;

    public ShooterCommand(ShooterSubsystem shooter, double speed) {
        this.shooter = shooter;
        this.speed = speed;

        addRequirements(shooter);
    }

    @Override
    public void initialize() {
        shooter.drive(speed);
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
