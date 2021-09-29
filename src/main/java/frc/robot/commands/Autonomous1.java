// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.ConsoleCommandConstants;
import frc.robot.libraries.ConsoleJoystick;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Vision;

public class Autonomous1 extends CommandBase {

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

  Command m_currentCommand;
  int m_waitCount;


  /** Creates a new Autonomous1. */
  public Autonomous1(ConsoleJoystick console, Drive drive, Hopper hopper, Intake intake, Shooter shooter, Vision vision) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_console = console;
    m_drive = drive;
    m_hopper = hopper;
    m_intake = intake;
    m_shooter = shooter;
    m_vision = vision;
    addRequirements(m_drive, m_hopper, m_intake, m_shooter, m_vision);

    m_lDrive1 = new AutoDriveDistance(AutoConstants.kDISTANCE_1_LEFT, 0, m_drive);
    m_mDrive1 = new AutoDriveDistance(AutoConstants.kDISTANCE_1_MIDDLE, 0, m_drive);
    m_rDrive1 = new AutoDriveDistance(AutoConstants.kDISTANCE_1_RIGHT, 0, m_drive);

    m_wait1 = new Wait(1);
    m_wait2 = new Wait(2);
    
    m_launch = new ParallelRaceGroup(
      new Wait(AutoConstants.kLAUNCH_TIMER),
      new ShootPowerCells(m_shooter, AutoConstants.kLAUNCH_LOWER_VOLTAGE, AutoConstants.kLAUNCH_UPPER_VOLTAGE)
      );

  }
  
  private void dashboardCmd(String cmdName) {
    SmartDashboard.putString("Auto Cmd Step", cmdName);
  }
  
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    m_positionSwitch = m_console.getROT_SW_0();

    switch (m_positionSwitch) {
      case 0: 
        m_drive1 = m_lDrive1;
        break;
      case 1:
        m_drive1 = m_mDrive1;
        break;
      case 2:
        m_drive1 = m_rDrive1;
        break;
      default:
        m_drive1 = m_mDrive1;
    }

    m_currentCommand = m_drive1;
    dashboardCmd("Auto Drive 1");
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
    dashboardCmd("Auto Done");
  }

  private void switchCommand(final Command cmd) {
    m_currentCommand.end(false);
    m_currentCommand = cmd;
    m_currentCommand.initialize();
  }


  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    if (m_currentCommand.isFinished() == false) {
      return false;
    }

    if (m_currentCommand == m_drive1) {
      if (m_console.getROT_SW_1() > 0) {
        dashboardCmd("Wait2");
        switchCommand(m_wait2);
        m_waitCount = 1;
        return false;
      } else {
        dashboardCmd("Launch 0");
        switchCommand(m_launch);
        return false;
      }
    }

    if (m_currentCommand == m_wait2
        || m_currentCommand == m_wait1) {
      if (m_waitCount >= m_console.getROT_SW_1()) {
        dashboardCmd("Launch +");
        switchCommand(m_launch);
        return false;
      } else {
        dashboardCmd("Wait1+");
        switchCommand(m_wait1);
        m_waitCount++;
        return false;
      }
    }

    if (m_currentCommand == m_launch) {
      return true;
    }

    return true;
  }
}
