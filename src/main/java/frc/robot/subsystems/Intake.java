/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
//import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class Intake extends SubsystemBase {

  private String m_ArmStatus;
  private String m_SpinnerStatus;

  private WPI_TalonSRX m_beltMotor;
  private DoubleSolenoid m_IntakeDeploy;
  // private WPI_TalonSRX m_harvesterMotor;
  /**
   * Creates a new Intake.
   */
  public Intake() {
     m_ArmStatus = "initial";
    m_SpinnerStatus = "initial";

    m_beltMotor = new WPI_TalonSRX(IntakeConstants.kBELT_MOTOR_PORT);
    m_beltMotor.configFactoryDefault();

    m_IntakeDeploy = new DoubleSolenoid(IntakeConstants.kINTAKE_DOWN, IntakeConstants.kINTAKE_UP);
  }

  public void ArmUp() {
    m_ArmStatus = "Moving Up";
    m_IntakeDeploy.set(Value.kReverse);
  }

  public void ArmDown() {
    m_ArmStatus = "Moving Down";
    m_IntakeDeploy.set(Value.kForward);
  }
  
  public void ArmStop() {
    m_ArmStatus = "Arm Stopped Moving";
  }

  public boolean isArmDown() {
    return true;
  }

  public void SpinnerOn() {
    // harvester on
    m_SpinnerStatus = "Spinning";
  }

  public void SpinnerOff() {
    // harvester off
    m_SpinnerStatus = "Not Spinning";
  }

  public void SpinnerReverse() {
    // harvester reverse
    m_SpinnerStatus = "Spinning in reverse";
  }
  
  public void CombineUp() {
    SpinnerOff();
    ArmUp();
  }
  /*
  public void CombineDown() {
    ArmDown();
    SpinnerOn();
  }*/
  /*
  public void CombineRun() {
    if (m_isArmDown == false){
      ArmDown();
      m_isArmDown = true;
    }
    SpinnerOn();
  }
  */
  public void CombineStop() {
    //ArmStop();
    SpinnerOff();
  }

  public void beltOn(){
    m_beltMotor.set(IntakeConstants.kBELT_MOTOR_SPEED);
    SmartDashboard.putBoolean("Feeder Belt", true);
  }

  public void beltOff(){
    m_beltMotor.stopMotor();
    SmartDashboard.putBoolean("Feeder Belt", false);
  }

  @Override
  public void periodic() {
    SmartDashboard.putString("Arm Status", m_ArmStatus);
    SmartDashboard.putString("Spinner Status", m_SpinnerStatus);
  }
}
