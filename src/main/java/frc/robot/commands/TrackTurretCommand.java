// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class TrackTurretCommand extends CommandBase {
  ShooterSubsystem m_shooterSubsystem;
  VisionSubsystem m_visionSubsystem;


  /** Creates a new TrackTurretCommand. */
  public TrackTurretCommand(ShooterSubsystem shooterSubsystem, VisionSubsystem visionSubsystem) {
    this.m_shooterSubsystem = shooterSubsystem;
    this.m_visionSubsystem = visionSubsystem;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_shooterSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_visionSubsystem.isTargetValid()) {
      double yawOffset = m_visionSubsystem.getYaw();

      // For >360 degree motion. Uncomment when we do this
      // if (m_shooterSubsystem.getTurretAngle() + yawOffset > Constants.TURRET_UPPER_LIMIT) {
      //   yawOffset = Constants.TURRET_LOWER_LIMIT - m_shooterSubsystem.getTurretAngle();
      // } else if (m_shooterSubsystem.getTurretAngle() + yawOffset < Constants.TURRET_LOWER_LIMIT) {
      //   yawOffset = Constants.TURRET_UPPER_LIMIT - m_shooterSubsystem.getTurretAngle();
      // }

      if (Math.abs(yawOffset) > Constants.TURRET_TOLERANCE) {
          m_shooterSubsystem.setTurretDelta(yawOffset);
      } else {
          m_shooterSubsystem.setTurretSpeed(0.0);
      }
    } else {
      m_shooterSubsystem.setTurretDelta(-m_shooterSubsystem.getTurretAngle());
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_shooterSubsystem.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
