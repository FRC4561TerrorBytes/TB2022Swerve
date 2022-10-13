// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class ShootCommand extends CommandBase {

  private double loops = 0;

  private ShooterSubsystem m_shooterSubsystem;
  private FeederSubsystem m_feederSubsystem;

  private double m_shooterRPM;

  /** Creates a new ShootCommand. */
  public ShootCommand(ShooterSubsystem shooterSubsystem, FeederSubsystem feederSubsystem, double rpm) {
    this.m_shooterSubsystem = shooterSubsystem;
    this.m_feederSubsystem = feederSubsystem;
    this.m_shooterRPM = rpm;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_shooterSubsystem, m_feederSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_shooterSubsystem.setFlywheelSpeed(m_shooterRPM);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (Math.abs(m_shooterSubsystem.getFlywheelSpeed() - m_shooterRPM) < Constants.FLYWHEEL_TOLERANCE) {
      loops++;
      if (loops > 2) {
        m_feederSubsystem.feederShoot();
      }
    } else {
      loops = 0;
      m_feederSubsystem.stop();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_shooterSubsystem.stop();
    m_feederSubsystem.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
