package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.IntakeSubsystem;
 
public class IntakeCommand extends Command {
 
    private final IntakeSubsystem intake;
    private final double speed;
 
    public IntakeCommand(IntakeSubsystem intake, double speed) {
        this.intake = intake;
        this.speed = speed;
 
        addRequirements(intake);
    }
 
    @Override
    public void execute() {
        intake.drive(speed);
    }
 
    @Override
    public void end(boolean interrupted) {
        intake.stop();
    }
 
    @Override
    public boolean isFinished() {
        return false; // tourne tant que le bouton est maintenu
    }
}
 
 