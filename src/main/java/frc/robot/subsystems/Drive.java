/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpiutil.math.MathUtil;
import frc.robot.Constants.DriveConstants;

//import java.util.function.DoubleSupplier;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
//import com.revrobotics.EncoderType;

//import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.SPI;
//import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.smartdashboard.SendableRegistry;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.controller.PIDController;

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
  private double m_leftEncoderSign;
  private double m_rightEncoderSign;
  private double m_headingSign;

  private AHRS m_gyroAndCollison;
  /*private PIDController m_turnAngle;
  private double m_dTurnAngleP;
  private double m_dTurnAngleI;
  private double m_dTurnAngleD;
  private double m_dTurnAngleTolerance;*/
  private double m_dAngle;
  private final DifferentialDriveOdometry m_driveOdometry;

  public PIDController m_drivePIDController;
  private double m_dDriveDistanceP;
	private double m_dDriveDistanceI;
	private double m_dDriveDistanceD;
	private double m_dDriveDistanceTolerance;
	private double m_dDistance;
  
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

    m_leftController1.setIdleMode(CANSparkMax.IdleMode.kBrake);
    m_leftController2.setIdleMode(CANSparkMax.IdleMode.kBrake);
    m_rightController1.setIdleMode(CANSparkMax.IdleMode.kBrake);
    m_rightController2.setIdleMode(CANSparkMax.IdleMode.kBrake);

    m_leftController1.setOpenLoopRampRate(DriveConstants.kRAMP_RATE);
    m_leftController2.setOpenLoopRampRate(DriveConstants.kRAMP_RATE);
    m_rightController1.setOpenLoopRampRate(DriveConstants.kRAMP_RATE);
    m_rightController2.setOpenLoopRampRate(DriveConstants.kRAMP_RATE);

    m_leftController1.setSmartCurrentLimit(DriveConstants.kCURRENT_LIMT);
    m_leftController2.setSmartCurrentLimit(DriveConstants.kCURRENT_LIMT);
    m_rightController1.setSmartCurrentLimit(DriveConstants.kCURRENT_LIMT);
    m_rightController2.setSmartCurrentLimit(DriveConstants.kCURRENT_LIMT);
    
    m_leftControlGroup = new SpeedControllerGroup(m_leftController1, m_leftController2);
    m_rightControlGroup = new SpeedControllerGroup(m_rightController1, m_rightController2);
    m_leftControlGroup.setInverted(DriveConstants.kIS_DRIVE_INVERTED);
    m_rightControlGroup.setInverted(DriveConstants.kIS_DRIVE_INVERTED);

    m_robotDrive = new DifferentialDrive(m_leftControlGroup, m_rightControlGroup);

    m_leftEncoder1 = m_leftController1.getEncoder();
    m_rightEncoder1 = m_rightController1.getEncoder();
    //m_leftEncoder1 = m_leftController1.getEncoder(EncoderType.kHallSensor, 4096);
    //m_rightEncoder1 = m_rightController1.getEncoder(EncoderType.kHallSensor, 4096);

    if (DriveConstants.kIS_DRIVE_INVERTED) {
      m_leftEncoderSign = -1;
      m_rightEncoderSign = 1;
      m_headingSign = 1;
    } else {
      m_leftEncoderSign = 1;
      m_rightEncoderSign = -1;
      m_headingSign = -1;
    }

    m_gyroAndCollison = new AHRS(SPI.Port.kMXP);
    m_driveOdometry = new DifferentialDriveOdometry(getRotation2dK());

    SendableRegistry.addLW(m_robotDrive, "Drive Base");
    SendableRegistry.addLW(m_gyroAndCollison, "NavX");

    m_dDriveDistanceP = DriveConstants.kDRIVE_P;
    m_dDriveDistanceI = DriveConstants.kDRIVE_I;
    m_dDriveDistanceD = DriveConstants.kDRIVE_D;
    m_dDriveDistanceTolerance = DriveConstants.kDRIVE_TOLERANCE;
    m_dDistance = getDashboardDistance();
    m_drivePIDController = new PIDController(m_dDriveDistanceP, m_dDriveDistanceI, m_dDriveDistanceD);
    m_drivePIDController.setTolerance(m_dDriveDistanceTolerance);
 
    m_dAngle = 0;

    //m_keepHeading = new PIDController(DriveConstants.kKEEP_HEADING_P, DriveConstants.kKEEP_HEADING_I, DriveConstants.kKEEP_HEADING_D);
		m_dSteeringHeading = 0;
    
  } 

  @Override
  public void periodic() {
    //arcadeDrive(Robot.m_robotContainer.getVelocity(), Robot.m_robotContainer.getHeading());

    m_driveOdometry.update(getRotation2dK(), getLeftDistance(), getRightDistance());
    SmartDashboard.putNumber("GyroK", getAngleK());
    SmartDashboard.putNumber("LeftEncdr:", getLeftDistance());
    SmartDashboard.putNumber("RightEncdr:", getRightDistance());
    SmartDashboard.putNumber("Odo Y" , m_driveOdometry.getPoseMeters().getTranslation().getY());
    SmartDashboard.putNumber("Odo X", m_driveOdometry.getPoseMeters().getTranslation().getX());
    SmartDashboard.putNumber("Odo Deg", m_driveOdometry.getPoseMeters().getRotation().getDegrees());
  
  }

  public void arcadeDrive(double velocity, double heading){
    double dDriveInvert = -1; //NTU We need to add a muipiter, if we want to flip around the drivices.
    m_robotDrive.arcadeDrive(velocity * dDriveInvert, heading);
  }

  public double getDistance() {
    return -(m_leftEncoder1.getPosition() - m_rightEncoder1.getPosition()) / 2;
  }

  private double getLeftDistance() {
    return m_leftEncoderSign * m_leftEncoder1.getPosition() * DriveConstants.kENCODER_DISTANCE_PER_PULSE_M;
  }

  private double getRightDistance() {
    return m_rightEncoderSign * m_rightEncoder1.getPosition() * DriveConstants.kENCODER_DISTANCE_PER_PULSE_M;
  }

  public double getHeading() {
    return Math.IEEEremainder(m_gyroAndCollison.getAngle(), 360);
  }

  public void resetEncoders() {
    //m_gyroAndCollison.reset();
    m_leftEncoder1.setPosition(0);
    m_rightEncoder1.setPosition(0);
  }

  private double getAngleK() {
    return m_gyroAndCollison.getAngle();
  }

  private Rotation2d getRotation2dK() {
    // note the negation of the angle is required because the wpilib convention
    // uses left positive rotation while gyros read right positive
    return Rotation2d.fromDegrees(-getAngleK());
  }

  public void resetDrive() {
    resetEncoders();
  }

  private void resetAngle() {
    m_gyroAndCollison.zeroYaw();
    // need to add reset of odometry and encoders
  }

  public void reset() {
    resetAngle();
    resetDrive();
  }

  public double getDashboardDistance() {

    m_dDistance = SmartDashboard.getNumber("DriveDistance", m_dDistance);
    m_dDriveDistanceP = SmartDashboard.getNumber("DriveDistanceP", m_dDriveDistanceP);
    m_dDriveDistanceI = SmartDashboard.getNumber("DriveDistanceI", m_dDriveDistanceI);
    m_dDriveDistanceD = SmartDashboard.getNumber("DriveDistanceD", m_dDriveDistanceD);
    m_dDriveDistanceTolerance = SmartDashboard.getNumber("DriveDistanceTolerance", m_dDriveDistanceTolerance);
    m_dDistance = SmartDashboard.getNumber("DriveDistance", 2);
    SmartDashboard.putNumber("DriveDistance", m_dDistance);
    SmartDashboard.putNumber("DriveDistanceP", m_dDriveDistanceP);
    SmartDashboard.putNumber("DriveDistanceI", m_dDriveDistanceI);
    SmartDashboard.putNumber("DriveDistanceD", m_dDriveDistanceD);
    SmartDashboard.putNumber("DriveDistanceTolerance", m_dDriveDistanceTolerance);
    SmartDashboard.putNumber("DriveDistance", m_dDistance);

    return -m_dDistance;
  }

  public void resetPIDDriveController() {
    m_drivePIDController.setPID(m_dDriveDistanceP, m_dDriveDistanceI, m_dDriveDistanceD);
    m_drivePIDController.setTolerance(m_dDriveDistanceTolerance);
  }

  public void initDriveController(double distance) {
    double encoderDistance = distance / DriveConstants.kENCODER_DISTANCE_PER_PULSE_M;
    m_drivePIDController.setSetpoint(encoderDistance);
    resetEncoders();
    m_drivePIDController.reset();
  }
  
  public void execDriveController(double rotation) {
    arcadeDrive(MathUtil.clamp(m_drivePIDController.calculate(getDistance()),-DriveConstants.kDRIVE_PID_LIMIT, DriveConstants.kDRIVE_PID_LIMIT), rotation);
  }

  public void endDriveController() {
   arcadeDrive(0, 0);
  }

  public boolean isDriveAtSetpoint() {
    return m_drivePIDController.atSetpoint();
  }
}
