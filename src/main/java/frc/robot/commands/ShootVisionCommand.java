// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.VisionSubsystem;


public class ShootVisionCommand extends CommandBase {
  private DrivetrainSubsystem m_driveSubsystem;
  private ShooterSubsystem m_shooterSubsystem;
  private VisionSubsystem m_visionSubsystem;
  private FeederSubsystem m_feederSubsystem;
  private int m_loops = 0;
  private int m_loopNum;
  /**
   * Shoot using vision
   * @param driveSubsystem drive subsystem
   * @param shooterSubsystem shooter subsystem
   * @param visionSubsystem vision subsystem
   * @param delay shoot delay in seconds
   * @param odometry whether to update drive odometry or not
   */
  public ShootVisionCommand(DrivetrainSubsystem driveSubsystem, ShooterSubsystem shooterSubsystem,
                            FeederSubsystem feederSubsystem, 
                            VisionSubsystem visionSubsystem, double delay) {
    this.m_driveSubsystem = driveSubsystem;
    this.m_shooterSubsystem = shooterSubsystem;
    this.m_feederSubsystem = feederSubsystem;
    this.m_visionSubsystem = visionSubsystem;
    this.m_loopNum = (int)Math.round(delay / (1.0/60.0));

    // Use addRequirements() here to declare subsystem dependencies
    addRequirements(m_driveSubsystem, m_shooterSubsystem, m_visionSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Only run if target is valid
    if (m_visionSubsystem.isTargetValid()) {
      // Run flywheel based on distance from target
      m_shooterSubsystem.setShooterVision(m_visionSubsystem.getDistance());

      // Aim turret toward target
      // Only run feeder if flywheel is at speed and robot is on target, else stop
      if (m_shooterSubsystem.isFlywheelAtSpeed(m_visionSubsystem.getDistance()) &&  m_visionSubsystem.isOnTarget()) {
        m_loops++;
        if (m_loops > m_loopNum) m_feederSubsystem.feederShoot();
      } else {
        m_loops = 0;
        m_feederSubsystem.stop();
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // Stop flywheel and feeder
    m_shooterSubsystem.stop();
    m_feederSubsystem.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

