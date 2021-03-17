/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import java.util.function.DoubleSupplier;
import java.util.function.IntSupplier;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.*;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class StartAutoCmd extends SequentialCommandGroup {
  /**
   * Creates a new StartAutoCmd.
   */
  public StartAutoCmd(Autonomous autonomous, Drive drive, Intake intake, Shooter shooter, Hopper hopper, Vision vision, IntSupplier pov1, IntSupplier pov2) {
    addCommands(new LoadAutoXML(autonomous, pov1, pov2));
    addCommands(new StartGetNextCmd(autonomous, drive, intake, hopper, shooter, vision));
  }
}
