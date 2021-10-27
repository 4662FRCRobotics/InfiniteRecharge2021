// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.AutoConstants;
import frc.robot.libraries.AutoPosition;
import frc.robot.libraries.AutoStepCommand;
import frc.robot.libraries.AutonomousCommand;
import frc.robot.libraries.ConsoleJoystick;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Vision;

public class AutoControl extends CommandBase {
  /** Creates a new AutoControl. */
  AutonomousCommand m_autoStepCommand;
  ConsoleJoystick m_console;
  Drive m_drive;
  Hopper m_hopper;
  Intake m_intake;
  Shooter m_shooter;
  Vision m_vision;

  int m_positionSwitch;

  AutoDriveDistance m_lDrive1;
  AutoDriveDistance m_lDrive2;
  AutoDriveDistance m_mDrive1;
  AutoDriveDistance m_mDrive2;
  AutoDriveDistance m_rDrive1;
  AutoDriveDistance m_rDrive2;
  AutoDriveDistance m_drive1;
  AutoDriveDistance m_drive2;

  Wait m_wait1;
  Wait m_wait2;
  Wait m_waitShoot;

  ParallelRaceGroup m_launch;

  DeployHarvester m_deployHarvester;

  String m_currentStepName;
  Command m_currentCommand;
  int m_waitCount;

  /* testing weird idea
  */
  int m_stepIndex = 0;
  String m_stepName [] = {AutoStepCommand.DRIVE1.name(),
                          AutoStepCommand.WAITLOOP.name(),
                          AutoStepCommand.LAUNCH1.name(),
                          AutoStepCommand.DRIVE2.name(),
                          AutoStepCommand.DEPINTAKE.name()
                          };
  BooleanSupplier m_stepSwitch[] = {() -> true,
                                    () -> true,
                                    () -> m_console.cnsl_btn_2.get(),
                                    () -> m_console.cnsl_btn_3.get(),
                                    () -> m_console.cnsl_btn_4.get()
                                    };

  /*
  iterate over arrays
  if (m_stepSwitch[idx].getAsBoolean()) {
    next command map commend for m_stepName[]
  } else {
    if last command done
    else recursive call to this for the next command - depends on logic
  } 
  }
  */                                  
  // end weird idea

  public AutoControl(AutoPosition position) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_autoStepCommand = new AutonomousCommand();
    switch (position) {
      case LEFT:
        m_autoStepCommand.addOption(AutoStepCommand.DRIVE1.name(), new AutoDriveDistance(AutoConstants.kDISTANCE_1_LEFT, 0, m_drive));
        m_autoStepCommand.addOption(AutoStepCommand.DRIVE2.name(), new AutoDriveDistance(AutoConstants.kDISTANCE_2_LEFT, 0, m_drive));
        break;
      case MIDDLE:
        m_autoStepCommand.addOption(AutoStepCommand.DRIVE1.name(), new AutoDriveDistance(AutoConstants.kDISTANCE_1_MIDDLE, 0, m_drive));
        m_autoStepCommand.addOption(AutoStepCommand.DRIVE2.name(), new AutoDriveDistance(AutoConstants.kDISTANCE_2_MIDDLE, 0, m_drive));
        break;
      case RIGHT:
        m_autoStepCommand.addOption(AutoStepCommand.DRIVE1.name(), new AutoDriveDistance(AutoConstants.kDISTANCE_1_RIGHT, 0, m_drive));
        m_autoStepCommand.addOption(AutoStepCommand.DRIVE2.name(), new AutoDriveDistance(AutoConstants.kDISTANCE_2_RIGHT, 0, m_drive));
        break;
      default:
        m_autoStepCommand.addOption(AutoStepCommand.DRIVE1.name(), new AutoDriveDistance(AutoConstants.kDISTANCE_1_MIDDLE, 0, m_drive));
        m_autoStepCommand.addOption(AutoStepCommand.DRIVE2.name(), new AutoDriveDistance(AutoConstants.kDISTANCE_2_MIDDLE, 0, m_drive));
        break;
    }

    m_autoStepCommand.addOption(AutoStepCommand.WAITLOOP.name(), new Wait(1));
    m_autoStepCommand.addOption(AutoStepCommand.END.name(), new AutoEnd());
    m_autoStepCommand.addOption(AutoStepCommand.LAUNCH1.name(), 
      new ParallelRaceGroup(
        new Wait(4),
        new ShootPowerCells(m_shooter, AutoConstants.kLAUNCH_LOWER_VOLTAGE, AutoConstants.kLAUNCH_LOWER_VOLTAGE),
        new SequentialCommandGroup(new Wait(1), new ShootHopperFeed(m_hopper))
    ));
    
  }

  private void dashboardCmd(String cmdName) {
    SmartDashboard.putString("Auto Cmd Step", cmdName);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_waitCount = 0;
    m_stepIndex = -1;
    m_currentStepName = getNextActiveCommand();
    m_currentCommand = m_autoStepCommand.getSelected(m_currentStepName);
    dashboardCmd(m_currentStepName);
    m_currentCommand.initialize();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_currentCommand.execute();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_currentCommand.end(interrupted);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    boolean areWeThereYet = true;
    if (m_currentCommand.isFinished() == false) {
      areWeThereYet = false;
    } else {
      areWeThereYet = stepNextCommand();
    }
    return areWeThereYet;
  }

  public boolean stepNextCommand() {
    /*
    step to the next command
    default to stopping the command (areWeThereYet = true)
    if in a wait loop (rotary switch port 1 value)
      repeat wait loop
    Get the next command to be run
    if at end 
      stop the command
    step to next command
      special setup for wait loop
    */

    boolean areWeThereYet = true;

    if (m_stepName[m_stepIndex] == AutoStepCommand.WAITLOOP.name()
        && m_waitCount < m_console.getROT_SW_1()) {
      switchCommand(m_currentCommand);
      m_waitCount++;
      areWeThereYet = false;
    } else {
      m_currentStepName = getNextActiveCommand();
      if (m_currentStepName == AutoStepCommand.END.name()) {
        areWeThereYet = true;
      } else {
        if (m_currentStepName == AutoStepCommand.WAITLOOP.name()) {
          m_waitCount = 1;
        } 
        dashboardCmd(m_currentStepName);
        switchCommand(m_autoStepCommand.getSelected(m_currentStepName));
        areWeThereYet = false;
      }
    }

    return areWeThereYet;
    
  }

  /*
    calls the end method for the previous command
    starts the next command
  */
  private void switchCommand(final Command cmd) {
    m_currentCommand.end(false);
    m_currentCommand = cmd;
    m_currentCommand.initialize();
  }

  /*
    getNextActiveCommand
    loops through step table until it gets a command marked "true" and not a zero count waitloop
    if the table index hits the end then the end command is returned
  */
  private String getNextActiveCommand(){

    String returnStepName = "";

    while (returnStepName == "") {
      m_stepIndex++;
      if (m_stepIndex >= m_stepName.length) {
        returnStepName = AutoStepCommand.END.name();
      } else {
        if (m_stepSwitch[m_stepIndex].getAsBoolean() == true) {
          if (m_stepName[m_stepIndex] == AutoStepCommand.WAITLOOP.name()) {
            if (m_console.getROT_SW_1() > 0) {
              returnStepName = m_stepName[m_stepIndex];
            }
          } else {
            returnStepName = m_stepName[m_stepIndex];
          }
        }
      }
    }

    return returnStepName;
  }

}
