// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
 
package frc.robot.SwerveAndAuto;
 
import frc.robot.MainConstants;
 
import java.io.File;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
 
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
 
import swervelib.SwerveDrive;
import swervelib.parser.SwerveParser;
 
import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

// Limelight helper (add LimelightHelpers.java to your project OR vendor dep)
import frc.robot.LimelightHelpers;
 
public class SwerveSubsystem extends SubsystemBase {
 
  private static final String LIMELIGHT_NAME = "limelight";
 
  private final File directory = new File(Filesystem.getDeployDirectory(), "swerve");
  private final AHRS m_gyro = new AHRS(NavXComType.kMXP_SPI);
 
  private SwerveDrive swerveDrive;
 
  public int tagCountLL = 0;

  public SwerveSubsystem() {
    try {
      // NOTE: Translation2d takes doubles in meters.
      Pose2d startingPose = new Pose2d(new Translation2d(1.0, 4.0), Rotation2d.fromDegrees(0));
 
      swerveDrive = new SwerveParser(directory).createSwerveDrive(
          MainConstants.MAX_SPEED,
          startingPose
      );
    } catch (Exception e) {
      throw new RuntimeException(e);
    }
 
    // Your original setting
    setupPathPlanner();
    swerveDrive.setHeadingCorrection(false);
  }
 
  @Override
  public void periodic() {

    LimelightHelpers.PoseEstimate mt1 =
        LimelightHelpers.getBotPoseEstimate_wpiBlue(LIMELIGHT_NAME);
swerveDrive.addVisionMeasurement(
          mt1.pose,
          mt1.timestampSeconds,
          VecBuilder.fill(.5, .5, 9999999)
      );
      tagCountLL = mt1.tagCount;

      SmartDashboard.putNumber("LL tagCount", mt1.tagCount);
      SmartDashboard.putNumber("LL X", mt1.pose.getX());
      SmartDashboard.putNumber("LL Y", mt1.pose.getY());
      SmartDashboard.putNumber("LL Yaw(deg)", mt1.pose.getRotation().getDegrees());

      SmartDashboard.putNumber("SwerveX", swerveDrive.getPose().getX());
      SmartDashboard.putNumber("SwerveY", swerveDrive.getPose().getY());
      SmartDashboard.putNumber("SwerveYAW", swerveDrive.getPose().getRotation().getDegrees());




 
    // 1) Update YAGSL odometry every loop (this is important)
    // YAGSL docs / javadocs: updateOdometry should be run every loop. :contentReference[oaicite:4]{index=4}
    swerveDrive.updateOdometry();
 
    // 2) Publish gyro info (your original)
    SmartDashboard.putNumber("NavX-Yaw", m_gyro.getYaw());
 
    // 3) Add Limelight Robot Localization as vision corrections (MegaTag1 style)
    addLimelightVisionMeasurementMegaTag1();


  }
 
  private void addLimelightVisionMeasurementMegaTag1() {
    boolean doRejectUpdate = false;
 
    LimelightHelpers.PoseEstimate mt1 =
        LimelightHelpers.getBotPoseEstimate_wpiBlue(LIMELIGHT_NAME); // 2024+ use wpiblue :contentReference[oaicite:5]{index=5}
 
    // Basic rejection logic (from Limelight doc example)
    if (mt1.tagCount == 0) {
      doRejectUpdate = true;
    }
 
    if (mt1.tagCount == 1 && mt1.rawFiducials != null && mt1.rawFiducials.length == 1) {
      if (mt1.rawFiducials[0].ambiguity > 0.7) {
        doRejectUpdate = true;
      }
      if (mt1.rawFiducials[0].distToCamera > 3.0) { // meters
        doRejectUpdate = true;
      }
    }
 
    if (!doRejectUpdate) {
      // Same intent as your line:
      // poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(.5,.5,9999999));
      // In YAGSL, you can pass std devs directly into addVisionMeasurement overload. :contentReference[oaicite:6]{index=6}
      swerveDrive.addVisionMeasurement(
          mt1.pose,
          mt1.timestampSeconds,
          VecBuilder.fill(.5, .5, 9999999)
      );
 
      SmartDashboard.putNumber("LL tagCount", mt1.tagCount);
      SmartDashboard.putNumber("LL X", mt1.pose.getX());
      SmartDashboard.putNumber("LL Y", mt1.pose.getY());
      SmartDashboard.putNumber("LL Yaw(deg)", mt1.pose.getRotation().getDegrees());

      SmartDashboard.putNumber("SwerveX", swerveDrive.getPose().getX());
      SmartDashboard.putNumber("SwerveY", swerveDrive.getPose().getY());
      SmartDashboard.putNumber("SwerveYAW", swerveDrive.getPose().getRotation().getDegrees());
    }
  }
 
  // --- Your existing drive wrappers ---
  public void driveFieldOriented(ChassisSpeeds velocity) {
    swerveDrive.driveFieldOriented(velocity);
  }
 
  public Command driveFieldOriented(Supplier<ChassisSpeeds> velocity) {
    return run(() -> swerveDrive.driveFieldOriented(velocity.get()));
  }
 
  public void driveRobotOriented(ChassisSpeeds velocity) {
    swerveDrive.drive(velocity);
  }
 
  public Command driveRobotOriented(Supplier<ChassisSpeeds> velocity) {
    return run(() -> swerveDrive.drive(velocity.get()));
  }
 
  // --- Helpful getters for turret aiming ---
  public Pose2d getPose() {
    return swerveDrive.getPose();
  }
 
  public Rotation2d getHeading() {
    return getPose().getRotation();
  }
 
  public SwerveDrive getSwerveDrive() {
    return swerveDrive;
  }

  public void zeroHeading() {
    m_gyro.reset(); // Remet l'angle du NavX à zéro
    SmartDashboard.putNumber("NavX-Yaw", 0);
  }

    // ---------------- PATHPLANNER ----------------

    public void setupPathPlanner() {
        try {
            RobotConfig config = RobotConfig.fromGUISettings();

            final boolean enableFeedforward = true;

            AutoBuilder.configure(
                    swerveDrive::getPose,
                    swerveDrive::resetOdometry,
                    swerveDrive::getRobotVelocity,

                    (speedsRobotRelative, moduleFeedForwards) -> {
                        if (enableFeedforward) {
                            swerveDrive.drive(
                                    speedsRobotRelative,
                                    swerveDrive.kinematics.toSwerveModuleStates(speedsRobotRelative),
                                    moduleFeedForwards.linearForces()
                            );
                        } else {
                            swerveDrive.setChassisSpeeds(speedsRobotRelative);
                        }
                    },

                    new PPHolonomicDriveController(
                            new PIDConstants(1.0, 0.0, 0.0),
                            new PIDConstants(1.0, 0.0, 0.0)
                    ),

                    config,

                    // 🔥 Flip path si RED
                    () -> DriverStation.getAlliance()
                            .map(a -> a == DriverStation.Alliance.Red)
                            .orElse(false),

                    this
            );

        } catch (Exception e) {
            e.printStackTrace();
        }
    }

}