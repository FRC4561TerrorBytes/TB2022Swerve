// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.RemoteFeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxLimitSwitch;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
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

  private CANSparkMax m_leftFlywheelMotor;
  private CANSparkMax m_rightFlywheelMotor;
  private CANSparkMax m_turretMotor;
  private TalonSRX m_hoodMotor;

  private SparkMaxPIDController m_leftFlywheelPIDController;
  private SparkMaxPIDController m_rightFlywheelPIDController;
  private TalonPIDConfig m_hoodConfig;

  /** Creates a new ShooterSubsystem. */
  public ShooterSubsystem(Hardware shooterHardware, TalonPIDConfig hoodConfig) {
    this.m_leftFlywheelMotor = shooterHardware.leftFlywheelMotor;
    this.m_rightFlywheelMotor = shooterHardware.rightFlywheelMotor;
    this.m_turretMotor = shooterHardware.turretMotor;
    this.m_hoodMotor = shooterHardware.hoodMotor;
    this.m_leftFlywheelPIDController = m_leftFlywheelMotor.getPIDController();
    this.m_rightFlywheelPIDController = m_rightFlywheelMotor.getPIDController();
    this.m_hoodConfig = hoodConfig;
    
    m_hoodConfig.initializeTalonPID(m_hoodMotor, FeedbackDevice.CTRE_MagEncoder_Absolute);
    
    m_leftFlywheelMotor.setIdleMode(IdleMode.kCoast);
    m_rightFlywheelMotor.setIdleMode(IdleMode.kCoast);
    m_turretMotor.setIdleMode(IdleMode.kBrake);
    m_hoodMotor.setNeutralMode(NeutralMode.Brake);
    
    m_leftFlywheelMotor.setInverted(false);
    m_rightFlywheelMotor.setInverted(true);
    m_turretMotor.setInverted(true);
    m_hoodMotor.setInverted(true);

    
  }

  public void shoot() {
    m_leftFlywheelMotor.set(0.6);
    m_rightFlywheelMotor.set(0.6);
  }

  public void setHoodSpeed(double speed) {
    m_hoodMotor.set(TalonSRXControlMode.PercentOutput, speed);
  }

  public void setHoodPosition(double angle) {
    angle = MathUtil.clamp(angle, 20, 40);
    m_hoodMotor.set(TalonSRXControlMode.MotionMagic, 0);
  }

  public void setTurretSpeed(double speed) {
    m_turretMotor.set(speed);
  }

  public void setTurretPosition(double angle) {
    
  }

  public void stop() {
    m_leftFlywheelMotor.stopMotor();
    m_rightFlywheelMotor.stopMotor();
    m_turretMotor.stopMotor();
    m_hoodMotor.set(TalonSRXControlMode.PercentOutput, 0.0);
  }

  //FIXME rm this
  public void printThing() {
    System.out.println(m_hoodMotor.getSelectedSensorPosition());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
