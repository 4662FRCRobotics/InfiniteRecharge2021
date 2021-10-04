// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Intake;

public class DeployHarvester extends CommandBase {
  /** Creates a new DeployHarvester. */
  Hopper m_hopper;
  Intake m_intake;

  public DeployHarvester(Hopper hopper, Intake intake) {
    m_hopper = hopper;
    m_intake = intake;
    addRequirements(m_hopper, m_intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_hopper.extendBeltFrame();
    m_intake.ArmDown();
  }

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
