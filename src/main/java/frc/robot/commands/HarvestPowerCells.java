/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Intake;

public class HarvestPowerCells extends CommandBase {
  /**
   * Creates a new HarvestPowerCells.
   */
  Hopper m_hopper;
  Intake m_intake;
  public HarvestPowerCells(Hopper hopper, Intake intake) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_hopper = hopper;
    m_intake = intake;
    addRequirements(m_intake, m_hopper);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (m_hopper.shouldIntakeTurnOn()) {
      m_intake.ArmDown();
      m_hopper.extendBeltFrame();
      m_intake.beltOn();
      m_hopper.setHopperMotorIntake();
    }
    //if the hopper is full just exit
    //otherwise extend intake and hopper and 
    //start intake and hopper feeds
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_intake.beltOff();
    m_hopper.setHopperMotorOff();
    if (!interrupted) {
      m_hopper.retractBeltFrame();
      m_intake.ArmUp();
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    //return false;
    return m_hopper.isBallAtShooter();
  }
}
