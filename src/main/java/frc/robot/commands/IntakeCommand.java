// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeCommand extends CommandBase {

  enum AllianceColor {
    Red, Blue
  }

  private IntakeSubsystem m_intakeSubsystem;
  private FeederSubsystem m_feederSubsystem;
  private AllianceColor m_alliance;

  /** Creates a new IntakeCommand. */
  public IntakeCommand(IntakeSubsystem intakeSubsystem, FeederSubsystem feederSubsystem, String allianceColor) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_intakeSubsystem = intakeSubsystem;
    this.m_feederSubsystem = feederSubsystem;

    if (allianceColor.equals("Red")) {
      m_alliance = AllianceColor.Red;
    } else if (allianceColor.equals("Blue")) {
      m_alliance = AllianceColor.Blue;
    }

    addRequirements(m_intakeSubsystem, m_feederSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_intakeSubsystem.intake();
    m_feederSubsystem.feederIntake();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_feederSubsystem.updateLimitSwitches();
    // Color currentColorDifference = m_intakeSubsystem.getColorDifference();
    // if (currentColorDifference.red > currentColorDifference.blue) {
    //   if (m_alliance != AllianceColor.Red && currentColorDifference.red > 0.01) {
    //     m_feederSubsystem.feederOuttake();
    //     return;
    //   }
    // } else {
    //   if (m_alliance != AllianceColor.Blue && currentColorDifference.blue > 0.01) {
    //     m_feederSubsystem.feederOuttake();
    //     return;
    //   }
    // }
    // m_feederSubsystem.feederIntake();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_intakeSubsystem.stop();
    m_feederSubsystem.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
