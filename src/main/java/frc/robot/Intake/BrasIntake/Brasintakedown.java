package frc.robot.Intake.BrasIntake;
 
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Intake.IntakeConstants;
import frc.robot.Intake.BrasIntake.BrasIntakeSubsystem;
 
/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class Brasintakedown extends Command {
  private final BrasIntakeSubsystem m_Brasintake;
  /** Creates a new PositionLiftScoreLv1. */
  public Brasintakedown(BrasIntakeSubsystem LiftSubsystem) {
    m_Brasintake = LiftSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(LiftSubsystem);
  }
 
  // Called when the command is initially scheduled.
  public void initialize() {
  m_Brasintake.setPositionTarget(IntakeConstants.Position1Count);
}
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
     m_Brasintake.setPositionTarget(IntakeConstants.Position1Count);
    }
 
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}
 
  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_Brasintake.atSetpoint();
  }
}