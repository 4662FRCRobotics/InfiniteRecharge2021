/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Shooter;

public class ShootPowerCells extends CommandBase {
  /**
   * Creates a new ShootPowerCells.
   */
  private Shooter m_shooter;
  //private Joystick m_console;
  private DoubleSupplier m_shootThrottle;
  private DoubleSupplier m_upperOffset;
  //private double m_throttle;
  private double m_lowerVolts;
  private double m_upperVolts;
  private boolean m_bIsManual;
  
  public ShootPowerCells(Shooter shooter, DoubleSupplier shootThrottle, DoubleSupplier upperOffset) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_shooter = shooter;
    //m_console = console;
    m_shootThrottle = shootThrottle;
    m_upperOffset = upperOffset;
    m_bIsManual = true;
    addRequirements(m_shooter);
  }

  public ShootPowerCells(Shooter shooter, double lowerVolts, double upperVolts) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_shooter = shooter;
    m_lowerVolts = lowerVolts;
    m_upperVolts = upperVolts;
    m_bIsManual = false;
    addRequirements(m_shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_bIsManual){
      m_shooter.setMotorOn(m_shootThrottle, m_upperOffset);
    } else {
      m_shooter.setMotor(m_lowerVolts, m_upperVolts);
    }
  
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
