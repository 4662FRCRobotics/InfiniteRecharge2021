/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Common;
import frc.robot.Constants.HopperConstants;

public class Hopper extends SubsystemBase{
  
  private WPI_TalonSRX m_hopperMotor;
  private boolean m_bIsHopperMotorOn;
  private DigitalInput m_shooterSensor;
  private DigitalInput m_intakeSensor;
  private DoubleSolenoid m_beltFramePiston;

  private boolean m_bIsAtShooter;  // true - sees emitter
  private boolean m_bIsAtIntake;  // false - emitter is obscured

  public Hopper() {
    m_hopperMotor = new WPI_TalonSRX(HopperConstants.kHOPPER_MOTOR_PORT);
    m_shooterSensor = new DigitalInput(HopperConstants.kSHOOTER_SENSOR_PORT);
    m_intakeSensor = new DigitalInput(HopperConstants.kINTAKE_SENSOR_PORT);
    m_beltFramePiston = new DoubleSolenoid(Common.kPCM_PORT, HopperConstants.kBELT_FRAME_OUT, HopperConstants.kBELT_FRAME_IN);
    
    m_hopperMotor.configFactoryDefault();

    m_bIsAtShooter = false;
    m_bIsAtIntake = false;
    m_bIsHopperMotorOn = false;
  }

  @Override
  public void periodic() {
    m_bIsAtIntake = m_intakeSensor.get();
    m_bIsAtShooter = m_shooterSensor.get();
    
    SmartDashboard.putBoolean("Shooter Sensor reading", m_bIsAtShooter);
    SmartDashboard.putBoolean("Intake Sensor reading", m_bIsAtIntake);
    
    
  }

  public void setHopperMotor(double speed) {
    m_hopperMotor.set(speed);
  }

  public void setHopperMotorOn() {
    m_hopperMotor.set(HopperConstants.kHOPPER_SPEED);
    m_bIsHopperMotorOn = true;
    SmartDashboard.putBoolean("Is Hopper Motor On", m_bIsHopperMotorOn);
  }

  public void setHopperMotorOff() {
    m_hopperMotor.stopMotor();
    m_bIsHopperMotorOn = false;
    SmartDashboard.putBoolean("Is Hopper Motor On", m_bIsHopperMotorOn);
  }

  public void extendBeltFrame() {
    m_beltFramePiston.set(Value.kForward);
  }

  public void retractBeltFrame() {
    m_beltFramePiston.set(Value.kReverse);
  }
  
  public boolean isHopperAligned(){
    return true;
  }

  public boolean isBallAtShooter() {
    return m_bIsAtShooter;
  }

  public boolean shouldIntakeTurnOn(){
    return !(m_bIsAtShooter && m_bIsAtIntake);
  }

  //public boolean shouldHopperFeed(){
  //  return m_bShooterSensorReading;
  //}
}

  