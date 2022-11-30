// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxLimitSwitch;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class FeederSubsystem extends SubsystemBase {

  private static class Hardware {
    private CANSparkMax middleFeederMotor;
    private CANSparkMax lowerFeederMotor;
    private CANSparkMax upperFeederMotor;
    private SparkMaxLimitSwitch lowerFeederSensor;
    private SparkMaxLimitSwitch middleFeederSensor;
    private DigitalInput upperFeederSensor;

    public Hardware(CANSparkMax lowerFeederMotor, CANSparkMax middleFeederMotor, CANSparkMax upperFeederMotor,
        SparkMaxLimitSwitch lowerFeederSensor, SparkMaxLimitSwitch middleFeederSensor,
        DigitalInput upperFeederSensor) {
      this.lowerFeederMotor = lowerFeederMotor;
      this.middleFeederMotor = middleFeederMotor;
      this.upperFeederMotor = upperFeederMotor;
      this.lowerFeederSensor = lowerFeederSensor;
      this.middleFeederSensor = middleFeederSensor;
      this.upperFeederSensor = upperFeederSensor;
    }
  }

  public static Hardware initializeHardware() {
    CANSparkMax lowerMotor = new CANSparkMax(Constants.FEEDER_LOWER_MOTOR, MotorType.kBrushless);
    CANSparkMax middleMotor = new CANSparkMax(Constants.FEEDER_MIDDLE_MOTOR, MotorType.kBrushless);
    CANSparkMax upperMotor = new CANSparkMax(Constants.FEEDER_UPPER_MOTOR, MotorType.kBrushless);
    return new Hardware(
      lowerMotor,
      middleMotor,
      upperMotor,
      lowerMotor.getForwardLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen),
      middleMotor.getForwardLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen),
      new DigitalInput(Constants.UPPER_FEEDER_SENSOR)
    );
  }

  private CANSparkMax m_lowerFeederMotor;
  private CANSparkMax m_middleFeederMotor;
  private CANSparkMax m_upperFeederMotor;
  private SparkMaxLimitSwitch m_lowerFeederSensor;
  private SparkMaxLimitSwitch m_middleFeederSensor;
  private DigitalInput m_upperFeederSensor;

  /** Creates a new FeederSubsystem. */
  public FeederSubsystem(Hardware feederHardware) {
    this.m_lowerFeederMotor = feederHardware.lowerFeederMotor;
    this.m_middleFeederMotor = feederHardware.middleFeederMotor;
    this.m_upperFeederMotor = feederHardware.upperFeederMotor;
    this.m_lowerFeederSensor = feederHardware.lowerFeederSensor;
    this.m_middleFeederSensor = feederHardware.middleFeederSensor;
    this.m_upperFeederSensor = feederHardware.upperFeederSensor;

    m_lowerFeederMotor.setIdleMode(IdleMode.kBrake);
    m_middleFeederMotor.setIdleMode(IdleMode.kBrake);
    m_upperFeederMotor.setIdleMode(IdleMode.kBrake);

    m_lowerFeederMotor.setInverted(true);
    m_middleFeederMotor.setInverted(false);
    m_upperFeederMotor.setInverted(false);

    m_lowerFeederSensor.enableLimitSwitch(false);
    m_middleFeederSensor.enableLimitSwitch(false);
  }

  public void updateLimitSwitches() {
    if (!m_upperFeederSensor.get()) {
      m_middleFeederSensor.enableLimitSwitch(true);
    }
    if (m_middleFeederSensor.isLimitSwitchEnabled() && m_middleFeederSensor.isPressed()) {
      m_lowerFeederSensor.enableLimitSwitch(true);
    }
  }

  public void feederIntake() {
    m_lowerFeederSensor.enableLimitSwitch(false);
    m_middleFeederSensor.enableLimitSwitch(false);

    m_lowerFeederMotor.set(Constants.FEEDER_SPEED);
    m_middleFeederMotor.set(Constants.FEEDER_SPEED / 2);
    m_upperFeederMotor.set(0.0);
  }

  public void feederOuttake() {
    m_lowerFeederSensor.enableLimitSwitch(false);
    m_middleFeederSensor.enableLimitSwitch(false);

    m_lowerFeederMotor.set(-Constants.FEEDER_SPEED);
    m_middleFeederMotor.set(-Constants.FEEDER_SPEED);
    m_upperFeederMotor.set(-Constants.FEEDER_SPEED);
  }

  public void feederShoot() {
    m_lowerFeederSensor.enableLimitSwitch(false);
    m_middleFeederSensor.enableLimitSwitch(false);

    m_lowerFeederMotor.set(Constants.FEEDER_SPEED);
    m_middleFeederMotor.set(Constants.FEEDER_SPEED);
    m_upperFeederMotor.set(Constants.FEEDER_SPEED);
  }

  public void stop() {
    m_lowerFeederMotor.stopMotor();
    m_middleFeederMotor.stopMotor();
    m_upperFeederMotor.stopMotor();

    m_lowerFeederSensor.enableLimitSwitch(false);
    m_middleFeederSensor.enableLimitSwitch(false);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putBoolean("Ball 1", !m_upperFeederSensor.get());
    SmartDashboard.putBoolean("Ball 2", m_middleFeederSensor.isPressed());
    SmartDashboard.putBoolean("Ball 3", m_lowerFeederSensor.isPressed());
  }
}
