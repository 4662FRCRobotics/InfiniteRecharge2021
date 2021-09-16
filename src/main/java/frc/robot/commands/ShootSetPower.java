// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Shooter;

public class ShootSetPower extends CommandBase {

  Shooter m_shooter;
  DoubleSupplier m_shootThrottle;
  DoubleSupplier m_upperOffset;

  /** Creates a new ShootSetPower. */
  public ShootSetPower(Shooter shooter, DoubleSupplier shootThrottle, DoubleSupplier upperOffset) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_shooter = shooter;
    m_shootThrottle = shootThrottle;
    m_upperOffset = upperOffset;
    addRequirements(shooter);
    m_shooter.setMotorOff();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_shooter.setVoltage(m_shootThrottle, m_upperOffset);
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
