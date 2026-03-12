package frc.robot.Climb;
import edu.wpi.first.wpilibj2.command.Command;

 
public class ClimbingCommand extends Command {
 
    private final ClimbSubsystem climb;
    private final double speed;
 
    public ClimbingCommand(ClimbSubsystem climb, double speed) {
        this.climb = climb;
        this.speed = speed;
 
        addRequirements(climb);
    }
 
    @Override
    public void execute() {
        climb.drive(speed);
    }
 
    @Override
    public void end(boolean interrupted) {
        climb.stop();
    }
 
    @Override
    public boolean isFinished() {
        return false; // tourne tant que le bouton est maintenu
    }
}
 
 