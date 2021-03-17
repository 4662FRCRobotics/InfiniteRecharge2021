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
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Vision;

public class ShootPowerCells extends CommandBase {
  /**
   * Creates a new ShootPowerCells.
   */
  private Shooter m_shooter;
  private Hopper m_hopper;
  private Vision m_vision;
  private Joystick m_driveStick;
  private double m_throttle;
  private boolean m_bIsAutonomous;
  public ShootPowerCells(Hopper hopper, Shooter shooter, Vision vision, Joystick driveStick) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_hopper = hopper;
    m_shooter = shooter;
    m_vision = vision;
    m_driveStick = driveStick;
    m_bIsAutonomous = false;
    addRequirements(m_hopper);
    addRequirements(m_shooter);
  }

  public ShootPowerCells(Hopper hopper, Shooter shooter, Vision vision) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_hopper = hopper;
    m_shooter = shooter;
    m_vision = vision;
    m_bIsAutonomous = true;
    addRequirements(m_hopper);
    addRequirements(m_shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_bIsAutonomous){
      m_throttle = 1;
    } else {
      m_throttle = m_driveStick.getThrottle();
    }
    //if (m_vision.isHighGoalAligned()){
      m_shooter.setMotorOn(m_driveStick.getThrottle());
    //} else {
      //m_shooter.setMotorOff();
    //}
    
    if (m_hopper.shouldHopperFeed()){
      m_hopper.setHopperMotorOn();
    } else {
      m_hopper.setHopperMotorOff();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_shooter.setMotorOff();
    m_hopper.setHopperMotorOff();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
