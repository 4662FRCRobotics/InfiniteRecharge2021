/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.DriveConstants;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.EncoderType;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.SpeedControllerGroup;

public class Drive extends SubsystemBase {
  /**
   * Creates a new Drive.
   */

  private CANSparkMax m_leftController1;
  private CANSparkMax m_leftController2;
  private CANSparkMax m_rightController1;
  private CANSparkMax m_rightController2;

  private SpeedControllerGroup m_leftControlGroup;
  private SpeedControllerGroup m_rightControlGroup;

  private CANEncoder m_leftEncoder1;
  private CANEncoder m_rightEncoder1;

  private DifferentialDrive m_robotDrive;

  private AHRS m_gyroAndCollison;
  /*private PIDController m_turnAngle;
  private double m_dTurnAngleP;
  private double m_dTurnAngleI;
  private double m_dTurnAngleD;
  private double m_dTurnAngleTolerance;*/
  private double m_dAngle;

  //private PIDController m_keepHeading;
  
  private volatile double m_dSteeringHeading;

  public Drive() {
    m_leftController1 = new CANSparkMax(DriveConstants.kLeftMotor1Port, CANSparkMaxLowLevel.MotorType.kBrushless);
    m_leftController2 = new CANSparkMax(DriveConstants.kLeftMotor2Port, CANSparkMaxLowLevel.MotorType.kBrushless);
    m_rightController1 = new CANSparkMax(DriveConstants.kRightMotor1Port, CANSparkMaxLowLevel.MotorType.kBrushless);
    m_rightController2 = new CANSparkMax(DriveConstants.kRightMotor2Port, CANSparkMaxLowLevel.MotorType.kBrushless);

    m_rightController1.restoreFactoryDefaults();
    m_rightController2.restoreFactoryDefaults();
    m_leftController1.restoreFactoryDefaults();
    m_leftController2.restoreFactoryDefaults();
    
    m_leftControlGroup = new SpeedControllerGroup(m_leftController1, m_leftController2);
    m_rightControlGroup = new SpeedControllerGroup(m_rightController1, m_rightController2);

    m_robotDrive = new DifferentialDrive(m_leftControlGroup, m_rightControlGroup);

    //m_leftEncoder1 = m_leftController1.getEncoder();
    //m_rightEncoder1 = m_rightController1.getEncoder();
    m_leftEncoder1 = new CANEncoder(m_leftController1, EncoderType.kHallSensor, 4096);
    m_rightEncoder1 = new CANEncoder(m_rightController1, EncoderType.kHallSensor, 4096);

    m_leftController1.setIdleMode(CANSparkMax.IdleMode.kCoast);
    m_leftController2.setIdleMode(CANSparkMax.IdleMode.kCoast);
    m_rightController1.setIdleMode(CANSparkMax.IdleMode.kCoast);
    m_rightController2.setIdleMode(CANSparkMax.IdleMode.kCoast);

    m_leftController1.setOpenLoopRampRate(DriveConstants.kRAMP_RATE);
    m_leftController2.setOpenLoopRampRate(DriveConstants.kRAMP_RATE);
    m_rightController1.setOpenLoopRampRate(DriveConstants.kRAMP_RATE);
    m_rightController2.setOpenLoopRampRate(DriveConstants.kRAMP_RATE);

    m_leftController1.setSmartCurrentLimit(DriveConstants.kCURRENT_LIMT);
    m_leftController2.setSmartCurrentLimit(DriveConstants.kCURRENT_LIMT);
    m_rightController1.setSmartCurrentLimit(DriveConstants.kCURRENT_LIMT);
    m_rightController2.setSmartCurrentLimit(DriveConstants.kCURRENT_LIMT);

    m_gyroAndCollison = new AHRS(SPI.Port.kMXP);

    //m_turnAngle = new PIDController(DriveConstants.kTURN_ANGLE_P, DriveConstants.kTURN_ANGLE_I, DriveConstants.kTURN_ANGLE_D);
    m_dAngle = 0;

    //m_keepHeading = new PIDController(DriveConstants.kKEEP_HEADING_P, DriveConstants.kKEEP_HEADING_I, DriveConstants.kKEEP_HEADING_D);
		m_dSteeringHeading = 0;
    
  } 

  @Override
  public void periodic() {
    //arcadeDrive(Robot.m_robotContainer.getVelocity(), Robot.m_robotContainer.getHeading());

      //double distancePerEncoderTic = DriveConstants.kGEARBOX_REDUCTION * (DriveConstants.kTIRE_SIZE * Math.PI);
      double distancePerEncoderTic = (DriveConstants.kTIRE_SIZE * Math.PI) / DriveConstants.kTIRE_SIZE / DriveConstants.kPULSE_PER_ROTATION;
      SmartDashboard.putNumber("leftencoder", m_leftEncoder1.getPosition() /*distancePerEncoderTic*/); // "/"
      SmartDashboard.putNumber("rightencoder", m_rightEncoder1.getPosition() /* distancePerEncoderTic*/); //"/"
      SmartDashboard.putNumber("Gyro", m_gyroAndCollison.getAngle());
  
  }

  public void arcadeDrive(double velocity, double heading){
    double dDriveInvert = -1; //NTU We need to add a muipiter, if we want to flip around the drivices.
    m_robotDrive.arcadeDrive(velocity * dDriveInvert, heading);
  }

  public double getDistance() {
    return -(m_leftEncoder1.getPosition() - m_rightEncoder1.getPosition()) / 2;
  }

  public double getHeading() {
    return Math.IEEEremainder(m_gyroAndCollison.getAngle(), 360);
  }

  public void reset() {
    m_gyroAndCollison.reset();
    m_leftEncoder1.setPosition(0);
    m_rightEncoder1.setPosition(0);
  }

}
