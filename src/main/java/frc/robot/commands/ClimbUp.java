/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.*;

public class ClimbUp extends CommandBase {
  private final Climb m_climb;

  /**
   * Creates a new ClimbUp.
   */
  public ClimbUp(Climb subsystem) {
    m_climb = subsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_climb);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_climb.climbBrakeRelease();
    m_climb.climbUp();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_climb.climbStop();
    m_climb.climbBrakeSet();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
