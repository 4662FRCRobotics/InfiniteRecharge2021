// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Hopper;

public class ShootHopperFeed extends CommandBase {

  private Hopper m_hopper;
  /** Creates a new ShootHopperFeed. */
  public ShootHopperFeed(Hopper hopper) {
    m_hopper = hopper;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_hopper);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_hopper.retractBeltFrame();
    //  if (m_hopper.shouldHopperFeed()){
    m_hopper.setHopperMotorLaunch();
  //  } else {
  //    m_hopper.setHopperMotorOff();
  //  }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_hopper.setHopperMotorOff();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
