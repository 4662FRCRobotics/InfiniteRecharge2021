/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import java.util.function.DoubleSupplier;
import java.util.function.IntSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.*;

public class LoadAutoXML extends CommandBase {
  private final Autonomous m_autonomous;
  private final IntSupplier m_pov1;
  private final IntSupplier m_pov2;
  /**
   * Creates a new LoadAutoXML.
   */
  public LoadAutoXML(Autonomous autonomous, IntSupplier pov1, IntSupplier pov2) {
    m_autonomous = autonomous;
    m_pov1 = pov1;
    m_pov2 = pov2;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_autonomous);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_autonomous.getXML(m_pov1, m_pov2);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
