/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.HopperConstants;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

public class OldHopper extends SubsystemBase {
  /**
   * Creates a new Hopper.
   */

  private WPI_TalonSRX m_hopperMotor;
  private boolean m_bIsHopperMotorOn;
  private DigitalInput m_shooterSensor;
  private DigitalInput m_intakeSensor;
  private DutyCycleEncoder m_hopperEncoder;
  //private DigitalInput m_hopperAtIntake = new DigitalInput(HopperConstants.kHOPPER_AT_INTAKE_PORT);

  private double m_hopperDistance;
  private double m_hopperDistanceTarget;

  private boolean m_bShooterSensorReading;  // true - sees emitter
  private boolean m_bIntakeSensorReading;  // false - emitter is obscured
  private boolean m_bIsHopperAtIntake;

  public OldHopper() {
    m_hopperMotor = new WPI_TalonSRX(HopperConstants.kHOPPER_MOTOR_PORT);
    m_shooterSensor = new DigitalInput(HopperConstants.kSHOOTER_SENSOR_PORT);
    m_intakeSensor = new DigitalInput(HopperConstants.kINTAKE_SENSOR_PORT);
    m_hopperEncoder = new DutyCycleEncoder(HopperConstants.kHOPPER_ENCODER_PORT);

    m_hopperEncoder.setDistancePerRotation(HopperConstants.kDISTANCE_PER_ROTATION);

    m_hopperDistance = 0.0;
    m_hopperDistanceTarget = 0.0;

    m_bShooterSensorReading = true;
    m_bIntakeSensorReading = true;
    m_bIsHopperAtIntake = false;
  }

  @Override
  public void periodic() {
    m_bIntakeSensorReading = m_intakeSensor.get();
    m_bShooterSensorReading = m_shooterSensor.get();
    m_bIsHopperAtIntake = isHopperAligned();

    m_hopperDistance = getEncoderDistance();

    SmartDashboard.putBoolean("Shooter Sensor reading", m_bShooterSensorReading);
    SmartDashboard.putBoolean("Intake Sensor reading", m_bIntakeSensorReading);
    SmartDashboard.putBoolean("Hopper aligned at intake", m_bIsHopperAtIntake);
    SmartDashboard.putNumber("Distance", m_hopperDistance);
    SmartDashboard.putNumber("Distance Target", m_hopperDistanceTarget);
  }

  public void zeroHopperEncoder(){
    m_hopperDistanceTarget = 0.0;
    m_hopperEncoder.reset();
  }

  public void setHopperMotor(double speed) {
    m_hopperMotor.set(speed);
  }

  public void setHopperMotorOn() {
    setHopperMotor(HopperConstants.kHOPPER_SPEED);
    m_bIsHopperMotorOn = true;
    SmartDashboard.putBoolean("Is Hopper Motor On", m_bIsHopperMotorOn);
  }

  public void setHopperMotorOff() {
    setHopperMotor(HopperConstants.kHOPPER_ZERO_SPEED);
    m_bIsHopperMotorOn = false;
    SmartDashboard.putBoolean("Is Hopper Motor On", m_bIsHopperMotorOn);
  }

  private double getEncoderDistance(){
    return m_hopperEncoder.getDistance();
  }

  public boolean isHopperAligned(){
    return Math.abs(m_hopperDistance - m_hopperDistanceTarget) < HopperConstants.kHOPPER_ENCODER_TOLERANCE; 
  }

  private boolean isBallAtShooter(){
    return !m_bShooterSensorReading;
  }

  private boolean isBallAtIntake(){
    return !m_bIntakeSensorReading;
  }

  public boolean shouldHopperTurnOn(){
    return (!isBallAtShooter()) && isBallAtIntake();
  }

  public void setTarget(){
    m_hopperDistanceTarget = Math.round(m_hopperDistance) - 1.0;  // target next slot
  }

  public boolean shouldHopperStop(){
    if (m_bIsHopperAtIntake && shouldHopperTurnOn()){
      setTarget();  // Keep going
    }

    return m_bIsHopperAtIntake && !shouldHopperTurnOn();
  }

  public boolean shouldIntakeTurnOn(){
    return !(isBallAtShooter() && isBallAtIntake());
  }

  public boolean shouldHopperFeed(){
    // turns on when power cell isn't at shooter OR hopper isn't aligned
    if (m_bShooterSensorReading && m_bIsHopperAtIntake){
      setTarget();  // Keep going if hopper aligned but ball not at shooter
    }

    return m_bShooterSensorReading || !m_bIsHopperAtIntake;  // !isBallAtShooter -> !!m_bShooterSensorReading
  }
}
