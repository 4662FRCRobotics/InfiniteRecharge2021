/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Shooter;

public class ShootPowerCells extends CommandBase {
  /**
   * Creates a new ShootPowerCells.
   */
  private Shooter m_shooter;
  private Joystick m_console;
  private double m_throttle;
  private boolean m_bIsAutonomous;
  
  public ShootPowerCells(Shooter shooter, Joystick console) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_shooter = shooter;
    m_console = console;
    m_bIsAutonomous = false;
    addRequirements(m_shooter);
  }

  public ShootPowerCells(Shooter shooter) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_shooter = shooter;
    m_bIsAutonomous = true;
    addRequirements(m_shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //m_hopper.retractBeltFrame();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    /*if (m_bIsAutonomous){
      m_throttle = 1;
    } else {
      m_throttle = m_console.getZ();
    }
    */
    //if (m_vision.isHighGoalAligned()){
      m_shooter.setMotorOn(m_console.getZ());
    //} else {
      //m_shooter.setMotorOff();
    //}
    
  
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_shooter.setMotorOff();
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
