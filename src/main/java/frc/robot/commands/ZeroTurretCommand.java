// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.revrobotics.SparkMaxLimitSwitch;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class ZeroTurretCommand extends CommandBase {

  private ShooterSubsystem m_shooterSubsystem;
  private IntakeSubsystem m_intakeSubsystem;

  private SparkMaxLimitSwitch farLimit;
  private SparkMaxLimitSwitch closeLimit;

  /** Creates a new ZeroTurretCommand. */
  public ZeroTurretCommand(ShooterSubsystem shooterSubsystem, IntakeSubsystem intakeSubsystem) {
    this.m_shooterSubsystem = shooterSubsystem;
    this.m_intakeSubsystem = intakeSubsystem;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_shooterSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    var limitSwitches = m_intakeSubsystem.getLimitSwitches();
    farLimit = limitSwitches[0];
    closeLimit = limitSwitches[1];

    m_shooterSubsystem.setTurretSpeed(0.15);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (farLimit.isPressed()) m_shooterSubsystem.setTurretSpeed(-0.15);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_shooterSubsystem.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return closeLimit.isPressed();
  }
}
