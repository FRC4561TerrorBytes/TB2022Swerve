// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.List;
import java.util.Map.Entry;

import org.apache.commons.math3.analysis.interpolation.SplineInterpolator;
import org.apache.commons.math3.analysis.polynomials.PolynomialSplineFunction;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
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

  private SparkPIDConfig m_leftFlywheelConfig;
  private SparkPIDConfig m_rightFlywheelConfig;
  private SparkPIDConfig m_turretConfig;
  private TalonPIDConfig m_hoodConfig;

  private PolynomialSplineFunction m_flywheelVisionCurve;
  private PolynomialSplineFunction m_hoodVisionCurve;
  private double m_minDistance;
  private double m_maxDistance;

  /** Creates a new ShooterSubsystem. */
  public ShooterSubsystem(Hardware shooterHardware, TalonPIDConfig hoodConfig, SparkPIDConfig flywheelConfig, SparkPIDConfig turretConfig, 
                          List<Entry<Double, FlywheelHoodSetpoint>> flywheelVisionMap) {
    this.m_leftFlywheelMotor = shooterHardware.leftFlywheelMotor;
    this.m_rightFlywheelMotor = shooterHardware.rightFlywheelMotor;
    this.m_turretMotor = shooterHardware.turretMotor;
    this.m_hoodMotor = shooterHardware.hoodMotor;
    this.m_leftFlywheelConfig = flywheelConfig;
    this.m_rightFlywheelConfig = flywheelConfig;
    this.m_hoodConfig = hoodConfig;
    this.m_turretConfig = turretConfig;
    
    m_hoodConfig.initializeTalonPID(m_hoodMotor, FeedbackDevice.CTRE_MagEncoder_Absolute);
    m_leftFlywheelConfig.initializeSparkPID(m_leftFlywheelMotor);
    m_rightFlywheelConfig.initializeSparkPID(m_rightFlywheelMotor);
    m_turretConfig.initializeSparkPID(m_turretMotor);
    
    m_leftFlywheelMotor.setIdleMode(IdleMode.kCoast);
    m_rightFlywheelMotor.setIdleMode(IdleMode.kCoast);
    m_turretMotor.setIdleMode(IdleMode.kBrake);
    m_hoodMotor.setNeutralMode(NeutralMode.Brake);
    
    m_leftFlywheelMotor.setInverted(false);
    m_rightFlywheelMotor.setInverted(true);
    m_turretMotor.setInverted(true);
    m_hoodMotor.setInverted(true);
    
    // Initialize shooter vision curves
    initializeFlywheelVisionCurve(flywheelVisionMap);
  }

  /**
   * Initialize vision curve spline functions
   * @param flywheelVisionMap List of distance/FlywheelSpeed pairs
   */
  private void initializeFlywheelVisionCurve(List<Entry<Double, FlywheelHoodSetpoint>> flywheelVisionMap) {
    double[] distances = new double[flywheelVisionMap.size()];
    double[] flywheelSpeeds = new double[flywheelVisionMap.size()];
    double[] hoodSetpoint = new double[flywheelVisionMap.size()];

    for (int i = 0; i < flywheelVisionMap.size(); i++) {
      distances[i] = flywheelVisionMap.get(i).getKey();
      flywheelSpeeds[i] = flywheelVisionMap.get(i).getValue().getFlywheelSpeed();
      hoodSetpoint[i] = flywheelVisionMap.get(i).getValue().getHoodSetpoint();
    }

    m_minDistance = distances[0];
    m_maxDistance = distances[distances.length - 1];

    m_flywheelVisionCurve = new SplineInterpolator().interpolate(distances, flywheelSpeeds);
    m_hoodVisionCurve = new SplineInterpolator().interpolate(distances, hoodSetpoint);
  }

  public void shoot() {
    m_leftFlywheelMotor.set(0.6);
    m_rightFlywheelMotor.set(0.6);
  }

  /**
   * Automatically sets the flywheel speed based on vision curve
   * <p>
   * NOTE: This method ALWAYS shoots high
   */
  public void setShooterVision(double distance) {
    double flywheelSpeed = m_flywheelVisionCurve.value(MathUtil.clamp(distance, m_minDistance, m_maxDistance));
  
    double hoodSetpoint = m_hoodVisionCurve.value(MathUtil.clamp(distance, m_minDistance, m_maxDistance));

    setFlywheel(flywheelSpeed);
    setHoodPosition(hoodSetpoint);
  }

  /**
   * Set flywheel to a speed
   * @param bigSpeed speed of big flywheel in RPM
   * @param smallSpeed speed of small flywheel in RPM
   */
  public void setFlywheel(double flywheelSpeed) {
    double flywheelSpeeds = MathUtil.clamp(flywheelSpeed, 0, Constants.NEO_MAX_RPM);
   
    m_rightFlywheelMotor.set(flywheelSpeeds);
    m_leftFlywheelMotor.set(flywheelSpeeds);
  }

  /**
   * Checks if flywheel is at set speed
   * @return True if flywheel is at speed else false
   */
  public boolean isFlywheelAtSpeed(double distance) {
    double flywheelTargetSpeed = m_flywheelVisionCurve.value(MathUtil.clamp(distance, m_minDistance, m_maxDistance));
    double rightFlywheelError = Math.abs(flywheelTargetSpeed - m_rightFlywheelMotor.getEncoder().getVelocity());
    double leftFlywheelError = Math.abs(flywheelTargetSpeed - m_leftFlywheelMotor.getEncoder().getVelocity());
    double hoodError = Math.abs(m_hoodMotor.getClosedLoopError());

    boolean isRightFlywheelAtSpeed = (rightFlywheelError < 1.0)
                                    && flywheelTargetSpeed != 0;
    boolean isLeftFlywheelAtSpeed = (leftFlywheelError < 1.0)
                                      && flywheelTargetSpeed != 0;
    boolean isHoodAtAngle = (hoodError < 1.0)
                              && m_hoodMotor.getClosedLoopTarget() != 0;
  
    return isRightFlywheelAtSpeed && isLeftFlywheelAtSpeed && isHoodAtAngle;
  }
  
  public void setHoodSpeed(double speed) {
    m_hoodMotor.set(TalonSRXControlMode.PercentOutput, speed);
  }

  public void setHoodPosition(double angle) {
    angle = MathUtil.clamp(angle, 20, 40);

    m_hoodMotor.set(TalonSRXControlMode.MotionMagic, angle);
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
    m_hoodMotor.set(ControlMode.MotionMagic, 0.0);
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
