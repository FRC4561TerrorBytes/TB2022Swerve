// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxLimitSwitch;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.PneumaticsControlModule;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase {

  private static class Hardware {
    private CANSparkMax feederMotor;
    private Solenoid leftSolenoid;
    private Solenoid rightSolenoid;

    public Hardware(CANSparkMax feederMotor, Solenoid leftSolenoid, Solenoid rightSolenoid) {
      this.feederMotor = feederMotor;
      this.leftSolenoid = leftSolenoid;
      this.rightSolenoid = rightSolenoid;
    }
  }

  public static Hardware initializeHardware() {
    return new Hardware(
      new CANSparkMax(Constants.INTAKE_ROLLER_MOTOR, MotorType.kBrushless),
      new Solenoid(Constants.PCM, PneumaticsModuleType.CTREPCM, Constants.LEFT_SOLENOID),
      new Solenoid(Constants.PCM, PneumaticsModuleType.CTREPCM, Constants.RIGHT_SOLENOID)
    );
  }

  private CANSparkMax m_feederMotor;
  private Solenoid m_leftSolenoid;
  private Solenoid m_rightSolenoid;

  /** Creates a new IntakeSubsyste. */
  public IntakeSubsystem(Hardware intakeHardware) {
    this.m_feederMotor = intakeHardware.feederMotor;
    this.m_leftSolenoid = intakeHardware.leftSolenoid;
    this.m_rightSolenoid = intakeHardware.rightSolenoid;

    m_feederMotor.setInverted(true);
  }

  public void setArmPosition(boolean armPosition) {
    m_leftSolenoid.set(armPosition);
    m_rightSolenoid.set(armPosition);
  }

  public void toggleArmPosition() {
    boolean newPosition = !m_leftSolenoid.get();
    setArmPosition(newPosition);
  }

  public void armDown() {
    setArmPosition(true);
  }

  public void armUp() {
    setArmPosition(false);
  }

  public void intake() {
    armDown();
    m_feederMotor.set(+Constants.INTAKE_SPEED);
  }

  public void outtake() {
    if (m_leftSolenoid.get())
      m_feederMotor.set(+Constants.INTAKE_SPEED);
  }

  public void stop() {
    m_feederMotor.stopMotor();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  /**
   * Get both limit switches for ShooterSubsystem
   */
  public SparkMaxLimitSwitch[] getLimitSwitches() {
    return new SparkMaxLimitSwitch[] {
      m_feederMotor.getForwardLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen),
      m_feederMotor.getReverseLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen)
    };
  }
}
