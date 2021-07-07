/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Vision;

public class FwdCameraTilt extends CommandBase {
  /**
   * Creates a new FwdCameraTilt.
   * 
   */
  private Vision m_vision;
  private DoubleSupplier m_cameraAngle;

  public FwdCameraTilt(Vision subsystem, DoubleSupplier cameraAngle) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_vision = subsystem;
    m_cameraAngle = cameraAngle;
    addRequirements(m_vision);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double cameraCntl = Math.abs(m_cameraAngle.getAsDouble() - 1);
    int iCameraCntl = (int) (cameraCntl * 90.0);
    m_vision.setAngle(iCameraCntl);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
