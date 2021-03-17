/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.ParamEnum;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.RemoteLimitSwitchSource;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;

public class Climb extends SubsystemBase {

  private Servo m_climbBrake;
  
  private boolean m_bIsClimbBrakeSet;
  private WPI_TalonSRX m_ClimbMotorFwd;
  private WPI_TalonSRX m_ClimbMotorInv;
  /**
   * Creates a new Climb.
   */
  public Climb() {
    m_bIsClimbBrakeSet = true;
    m_ClimbMotorFwd = new WPI_TalonSRX(ClimberConstants.kCLIMBER_FWD_PORT);
    m_ClimbMotorInv = new WPI_TalonSRX(ClimberConstants.kCLIMBER_INV_PORT);
    m_ClimbMotorFwd.configFactoryDefault();
    m_ClimbMotorInv.configFactoryDefault();

    m_ClimbMotorFwd.setNeutralMode(NeutralMode.Brake);
    m_ClimbMotorInv.setNeutralMode(NeutralMode.Brake);

    m_ClimbMotorFwd.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen, 0);
    m_ClimbMotorFwd.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen, 0);
    
    m_ClimbMotorInv.follow(m_ClimbMotorFwd);

    m_ClimbMotorFwd.setInverted(false);
    m_ClimbMotorInv.setInverted(InvertType.InvertMotorOutput);
    // flashes same color as master but reverses output as advertised
    m_climbBrake = new Servo(ClimberConstants.kCLIMBER_BRAKE_PORT);

  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("Climb Brake", m_bIsClimbBrakeSet);
    //Display the state of the Climb brake(s) on SmartDashboard.
  }
  private void setClimbMotor(double climbSpeed){
    SmartDashboard.putNumber("Climb Speed", climbSpeed);
    m_ClimbMotorFwd.set(climbSpeed);
    //Display the speed of the Climb motors on SmartDashboard.
  }
  public void climbUp(){
    setClimbMotor(ClimberConstants.kCLIMB_UP_SPEED);
  }
  public void climbDown(){
    setClimbMotor(ClimberConstants.kCLIMB_DOWN_SPEED);
  }
  public void climbStop(){
    setClimbMotor(ClimberConstants.kCLIMB_STOP);
  }

  public void climbBrakeSet(){
    m_climbBrake.setPosition(ClimberConstants.kCLIMB_BRAKE_CLOSE_ANGLE);
  }
  public void climbBrakeRelease(){
    m_climbBrake.setPosition(ClimberConstants.kCLIMB_BRAKE_OPEN_ANGLE);
  }
}
