// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class TaxiCommand extends CommandBase {

  private double m_loops = 0;
  private int m_direction;
  private DriveSubsystem m_driveSubsystem;

  /** Creates a new TaxiCommand. */
  public TaxiCommand(DriveSubsystem driveSubsystem, int direction) {
    this.m_driveSubsystem = driveSubsystem;
    this.m_direction = direction;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_driveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_driveSubsystem.drive(new ChassisSpeeds(-0.5, -0.5 * m_direction, 0.0));
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_loops++;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_driveSubsystem.drive(new ChassisSpeeds());
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_loops > 60 * 5;
  }
}
