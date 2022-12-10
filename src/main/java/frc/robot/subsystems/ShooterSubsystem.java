// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.SparkPIDConfig;
import frc.robot.TalonPIDConfig;


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

  public static class FlywheelHoodSetpoint {
    private double m_flywheelSpeed = 0.0;
    private double m_hoodSetpoint = 0.0;
    /**
     * Flywheel speed object
     * @param bigFlywheelSpeed big flywheel speed in RPM
     * @param hoodSetpoint hood setpoint from 0 - 1.0 (ticks)
     */
    public FlywheelHoodSetpoint(double flywheelSpeed, double hoodSetpoint) {
      this.m_flywheelSpeed = flywheelSpeed;
      this.m_hoodSetpoint = hoodSetpoint;
    }

    public double getFlywheelSpeed() {
      return m_flywheelSpeed;
    }

    public double getHoodSetpoint() {
      return m_hoodSetpoint;
    }
  }

  private CANSparkMax m_leftFlywheelMotor;
  private CANSparkMax m_rightFlywheelMotor;
  private CANSparkMax m_turretMotor;
  private TalonSRX m_hoodMotor;

  private SparkMaxPIDController m_leftPIDController;
  private SparkMaxPIDController m_rightPIDController;
  private SparkMaxPIDController m_turretPIDController;

  private SparkPIDConfig m_leftFlywheelConfig;
  private SparkPIDConfig m_rightFlywheelConfig;
  private SparkPIDConfig m_turretConfig;
  private TalonPIDConfig m_hoodConfig;

  /** Creates a new ShooterSubsystem. */
  public ShooterSubsystem(Hardware shooterHardware, TalonPIDConfig hoodConfig, SparkPIDConfig flywheelConfig, SparkPIDConfig turretConfig) {
    this.m_leftFlywheelMotor = shooterHardware.leftFlywheelMotor;
    this.m_rightFlywheelMotor = shooterHardware.rightFlywheelMotor;
    this.m_turretMotor = shooterHardware.turretMotor;
    this.m_hoodMotor = shooterHardware.hoodMotor;
    this.m_leftFlywheelConfig = flywheelConfig;
    this.m_rightFlywheelConfig = flywheelConfig;
    this.m_hoodConfig = hoodConfig;
    this.m_turretConfig = turretConfig;
    
    m_hoodConfig.initializeTalonPID(m_hoodMotor, FeedbackDevice.CTRE_MagEncoder_Relative);
    m_leftFlywheelConfig.initializeSparkPID(m_leftFlywheelMotor);
    m_rightFlywheelConfig.initializeSparkPID(m_rightFlywheelMotor);
    m_turretConfig.initializeSparkPID(m_turretMotor);

    this.m_leftPIDController = m_leftFlywheelMotor.getPIDController();
    this.m_rightPIDController = m_rightFlywheelMotor.getPIDController();
    this.m_turretPIDController = m_turretMotor.getPIDController();
    
    m_leftFlywheelMotor.setIdleMode(IdleMode.kCoast);
    m_rightFlywheelMotor.setIdleMode(IdleMode.kCoast);
    m_turretMotor.setIdleMode(IdleMode.kBrake);
    m_hoodMotor.setNeutralMode(NeutralMode.Brake);
    
    m_leftFlywheelMotor.setInverted(false);
    m_rightFlywheelMotor.setInverted(true);
    m_turretMotor.setInverted(true);
    m_hoodMotor.setInverted(true);
  }

  public void setFlywheel(double speed) {
    m_leftFlywheelMotor.set(speed);
    m_rightFlywheelMotor.set(speed);
  }

  
  public void setFlywheelSpeed(double rpm) {
    m_leftPIDController.setReference(rpm, ControlType.kSmartVelocity);
    m_rightPIDController.setReference(rpm, ControlType.kSmartVelocity);
  }

  public double getFlywheelSpeed() {
    return m_rightFlywheelMotor.getEncoder().getVelocity();
  }
  
  public void setHoodSpeed(double speed) {
    m_hoodMotor.set(TalonSRXControlMode.PercentOutput, speed);
  }

  public void setHoodPosition(double angle) {
    angle = MathUtil.clamp(angle, 20, 40);
    double setpoint = (angle - 20) / 20 * Constants.HOOD_UPPER_LIMIT;
    m_hoodMotor.set(TalonSRXControlMode.MotionMagic, setpoint);
  }

  public void setTurretSpeed(double speed) {
    m_turretMotor.set(speed);
  }

  public void setTurretDelta(double angleDelta) {
    double currentAngle = m_turretMotor.getEncoder().getPosition();
    if (Math.abs(angleDelta) < Constants.TURRET_TOLERANCE) return;
    m_turretPIDController.setReference(currentAngle + angleDelta, ControlType.kPosition);
  }

  public double getTurretAngle() {
    return m_turretMotor.getEncoder().getPosition();
  }

  public void resetTurretEncoder() {
    m_turretMotor.getEncoder().setPosition(-2.2);
  }

  public void stop() {
    m_leftFlywheelMotor.stopMotor();
    m_rightFlywheelMotor.stopMotor();
    m_turretMotor.stopMotor();
    m_hoodMotor.set(ControlMode.MotionMagic, 0.0);
    m_hoodMotor.set(TalonSRXControlMode.PercentOutput, 0.0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Flywheel speed", getFlywheelSpeed());
    SmartDashboard.putNumber("Turret position", m_turretMotor.getEncoder().getPosition());
  }
}
