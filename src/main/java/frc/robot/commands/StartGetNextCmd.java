/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.*;

public class StartGetNextCmd extends CommandBase {
  Autonomous m_autonomous;
  Drive m_drive;
  Intake m_intake;
  Hopper m_hopper;
  Shooter m_shooter;
  Vision m_vision;

  /**
   * Creates a new StartGetNextCmd.
   */
  public StartGetNextCmd(Autonomous autonomous, Drive drive, Intake intake, Hopper hopper, Shooter shooter, Vision vision) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_autonomous = autonomous;
    m_drive = drive;
    m_intake = intake;
    m_hopper = hopper;
    m_shooter = shooter;
    m_vision = vision;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Command commandGroup = (Command) new GetNextAutoCmd(m_autonomous, m_drive, m_intake, m_shooter, m_hopper, m_vision);
    commandGroup.schedule();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
