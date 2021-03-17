/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.*;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class AutoShoot extends SequentialCommandGroup {
  Shooter m_shooter;
  Vision m_vision;
  Drive m_drive;
  Hopper m_hopper;

  /**
   * Creates a new AutoShoot.
   */
  public AutoShoot(Double time, Shooter shooter, Vision vision, Drive drive, Hopper hopper) {
    // Add your commands in the super() call, e.g.
    // super(new FooCommand(), new BarCommand());
    super(new AlignedShooter(vision, drive), new ShootPowerCellTime(time, hopper, shooter, vision) );
  }
}
