package frc.robot.Shooter.Actuator;


import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ActuatorSubsystem extends SubsystemBase {

    private final Servo m_actuator = new Servo(0); // PWM port 0
    private final double MIN_ANGLE = 40;   // rétracté
    private final double MAX_ANGLE = 60; // étendu
    private double currentAngle = MIN_ANGLE;

    public ActuatorSubsystem() {
        m_actuator.setAngle(currentAngle); // position initiale
    }

    /* Étend l'actionneur à sa position maximale 
    public void extend() {
        currentAngle = MAX_ANGLE;
        m_actuator.setAngle(currentAngle);
    }

    /* Rétracte l'actionneur à sa position minimale */
    public void retract() {
        currentAngle = MIN_ANGLE;
        m_actuator.setAngle(currentAngle);
    }

    /** Déplace l'actionneur à une position intermédiaire en pourcentage **/
    public void setPositionPercent(double percent) {
        percent = Math.max(0, Math.min(1, percent)); // clamp 0–1
        currentAngle = MIN_ANGLE + percent * (MAX_ANGLE - MIN_ANGLE);
        m_actuator.setAngle(currentAngle);
    }

    /** Déplace progressivement pour plus de douceur **/
    public void moveIncrement(double deltaPercent) {
        double percent = (currentAngle - MIN_ANGLE) / (MAX_ANGLE - MIN_ANGLE);
        setPositionPercent(percent + deltaPercent);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Actuator Angle", currentAngle);
    }
} 