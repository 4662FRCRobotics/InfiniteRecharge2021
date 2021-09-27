// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ConsoleCommandConstants;
import frc.robot.libraries.ConsoleJoystick;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Vision;

public class Autonomous1 extends CommandBase {

  ConsoleJoystick m_console;
  Drive m_drive;
  Hopper m_hopper;
  Intake m_intake;
  Shooter m_shooter;
  Vision m_vision;

  /** Creates a new Autonomous1. */
  public Autonomous1(ConsoleJoystick console, Drive drive, Hopper hopper, Intake intake, Shooter shooter, Vision vision) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_console = console;
    m_drive = drive;
    m_hopper = hopper;
    m_intake = intake;
    m_shooter = shooter;
    m_vision = vision;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
