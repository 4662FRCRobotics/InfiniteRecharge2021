/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class Shooter extends SubsystemBase {
  /**
   * Creates a new Shooter.
   * okay
   */

  private WPI_TalonSRX m_shooterMotor0;
  private WPI_TalonSRX m_shooterMotor1;
  
  private boolean m_bIsMotorOn;

  public Shooter() {
    m_shooterMotor0 = new WPI_TalonSRX(ShooterConstants.kSHOOTER_MOTOR0_PORT);
    m_shooterMotor1 = new WPI_TalonSRX(ShooterConstants.kSHOOTER_MOTOR1_PORT);

    m_shooterMotor0.configFactoryDefault();
    m_shooterMotor1.configFactoryDefault();

    m_bIsMotorOn = false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    
  }

  private void setMotor(double speed){
    m_shooterMotor0.set(speed * ShooterConstants.kSHOOTER_DIRECTION);
    m_shooterMotor1.set(speed * -ShooterConstants.kSHOOTER_DIRECTION);
  }

  public void setMotorOn(double throttle){
    double adjustedThrottle = 2 / (throttle + 3);
    setMotor(ShooterConstants.kSHOOTER_SPEED * adjustedThrottle);
    m_bIsMotorOn = true;
    SmartDashboard.putBoolean("Shooter Motor", m_bIsMotorOn);
  }

  public void setMotorOff(){
    setMotor(ShooterConstants.kSHOOTER_ZERO_SPEED);
    m_bIsMotorOn = false;
    SmartDashboard.putBoolean("Shooter Motor", m_bIsMotorOn);
  }
}
