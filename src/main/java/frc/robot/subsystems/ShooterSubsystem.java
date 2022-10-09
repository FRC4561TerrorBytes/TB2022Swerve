// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxLimitSwitch;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShooterSubsystem extends SubsystemBase {

  private static class Hardware {
    private CANSparkMax leftFlywheelMotor;
    private CANSparkMax rightFlywheelMotor;
    private CANSparkMax turretMotor;
    private TalonSRX hoodMotor;

    public Hardware(CANSparkMax leftFlywheelMotor, CANSparkMax rightFlywheelMotor, CANSparkMax turretMotor,
        TalonSRX hoodMotor) {
      this.leftFlywheelMotor = leftFlywheelMotor;
      this.rightFlywheelMotor = rightFlywheelMotor;
      this.turretMotor = turretMotor;
      this.hoodMotor = hoodMotor;
    }
  }

  public static Hardware initializeHardware() {
    return new Hardware(
      new CANSparkMax(Constants.LEFT_FLYWHEEL_MOTOR, MotorType.kBrushless),
      new CANSparkMax(Constants.RIGHT_FLYWHEEL_MOTOR, MotorType.kBrushless),
      new CANSparkMax(Constants.TURRET_MOTOR, MotorType.kBrushless),
      new TalonSRX(Constants.HOOD_MOTOR)
    );
  }

  private CANSparkMax m_leftFlywheelMotor;
  private CANSparkMax m_rightFlywheelMotor;
  private CANSparkMax m_turretMotor;
  private TalonSRX m_hoodMotor;

  /** Creates a new ShooterSubsystem. */
  public ShooterSubsystem(Hardware shooterHardware) {
    this.m_leftFlywheelMotor = shooterHardware.leftFlywheelMotor;
    this.m_rightFlywheelMotor = shooterHardware.rightFlywheelMotor;
    this.m_turretMotor = shooterHardware.turretMotor;
    this.m_hoodMotor = shooterHardware.hoodMotor;

    m_leftFlywheelMotor.setIdleMode(IdleMode.kCoast);
    m_rightFlywheelMotor.setIdleMode(IdleMode.kCoast);
    m_turretMotor.setIdleMode(IdleMode.kBrake);
    
    m_leftFlywheelMotor.setInverted(false);
    m_rightFlywheelMotor.setInverted(true);
    m_turretMotor.setInverted(true);
  }

  public void shoot() {
    m_leftFlywheelMotor.set(0.6);
    m_rightFlywheelMotor.set(0.6);
  }

  public void stop() {
    m_leftFlywheelMotor.stopMotor();
    m_rightFlywheelMotor.stopMotor();
    m_turretMotor.stopMotor();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
