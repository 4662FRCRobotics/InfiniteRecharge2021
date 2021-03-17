/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.Drive;


// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class TurnToAngle extends PIDCommand {
  private final Drive m_drive;
  /**
   * Creates a new TurnToAngle.
   */
  public TurnToAngle(double targetAngle, Drive drive) {
    super(
        // The controller that the command will use
        new PIDController(DriveConstants.kTURN_ANGLE_P, DriveConstants.kTURN_ANGLE_I, DriveConstants.kTURN_ANGLE_D),
        // This should return the measurement
        drive::getHeading,
        // This should return the setpoint (can also be a constant)
        targetAngle,
        // This uses the output
        output -> drive.arcadeDrive(0, output),
          // Use the output here
        drive);
    // Use addRequirements() here to declare subsystem dependencies.
    // Configure additional PID options by calling `getController` here.
    getController().enableContinuousInput(-180, 180);

    getController().setTolerance(DriveConstants.kTURN_ANGLE_TOLERANCE, DriveConstants.kTURN_ANGLE_TOLERANCE_DEG_PER_S);
    m_drive = drive;
  } 
  public void initialize() {
    // Get everything in a safe starting state.
    m_drive.reset();
    super.initialize();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return getController().atSetpoint();
  }
}
