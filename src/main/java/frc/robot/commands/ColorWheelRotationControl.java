/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ContestantConstants;
import frc.robot.subsystems.WheelOfFortuneRotator;

public class ColorWheelRotationControl extends CommandBase {
  /**
   * Creates a new ColorWheelRotationControl.
   */
  private WheelOfFortuneRotator m_contestant;
  
  public ColorWheelRotationControl(WheelOfFortuneRotator contestant ) {
    m_contestant = contestant;
    addRequirements(m_contestant);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_contestant.setColorWheelMotor(ContestantConstants.kROTATION_MOTOR_SPEED);
    SmartDashboard.putBoolean("Rotation Control", true);
    m_contestant.detectColorChange();
    m_contestant.zeroColorChangeCount();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_contestant.detectColorChange();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_contestant.setColorWheelMotor(ContestantConstants.kZERO_SPEED);
    SmartDashboard.putBoolean("Rotation Control", false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_contestant.limitReached();
  }
}
