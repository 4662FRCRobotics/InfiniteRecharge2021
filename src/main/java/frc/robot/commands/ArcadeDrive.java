/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Drive;

import java.util.function.DoubleSupplier;

public class ArcadeDrive extends CommandBase {
  /**
   * Creates a new ArcadeDrive.
   */
  
  private final Drive m_drive;
  private final DoubleSupplier m_forward;
  private final DoubleSupplier m_rotation;
  private final DoubleSupplier m_speed;
  private final DoubleSupplier m_turnRate;

  public ArcadeDrive(Drive subsystem, DoubleSupplier forward, DoubleSupplier rotation, DoubleSupplier speed, DoubleSupplier turnRate) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_drive = subsystem;
    m_forward = forward;
    m_rotation = rotation;
    m_speed = speed;
    m_turnRate = turnRate;
    addRequirements(m_drive);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double speed = 1 - ((m_speed.getAsDouble() + 1) * 0.25);
    m_drive.arcadeDrive(m_forward.getAsDouble() * 2 / speed, m_rotation.getAsDouble() * 2 / (m_turnRate.getAsDouble() + 3));
  }

}

