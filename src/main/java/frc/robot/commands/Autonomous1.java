// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.AutoConstants;
import frc.robot.libraries.ConsoleJoystick;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Vision;

/* Autonomous Command
  This follows the standard Command/CommandBase pattern of initialize/exec/isFinished/end
  The specific autonomous step commands are instantiated and constructed
  Initialize starts the first command of the standard pattern and saves it as the currentCommand value
  Exec invokes the currentCommand.exec
  End invokes the currentCommand.end
  IsFinished does the major logic of stepping the autonomous pattern individual commands 
    in addition, checking appropriate switches (either rotary of digital) from the drive station console 
    provides field-side state selection of the actual steps to be run.
  Rotary switch 1 - sets the starting position of the robot as viewed from the team drive station
    0 - left
    1 - middle
    2 - right
    3 - default to middle pattern
  Rotary switch 2 - sets a delay before starting to launch Power Cells at the target - let other alliance robots go first if they want
    0 - no pause
    1 - two seconds
    2 - add one second per step from here on
  Button 2 - true to launch
  Button 3 - true to second drive
  Button 4 - true to deploy harvester

*/
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

  DeployHarvester m_deployHarvester;

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
    m_lDrive2 = new AutoDriveDistance(AutoConstants.kDISTANCE_2_LEFT, AutoConstants.kTURN_2_LEFT, m_drive);
    m_mDrive2 = new AutoDriveDistance(AutoConstants.kDISTANCE_2_MIDDLE, AutoConstants.kTURN_2_MIDDLE, m_drive);
    m_rDrive2 = new AutoDriveDistance(AutoConstants.kDISTANCE_2_RIGHT, AutoConstants.kTURN_2_RIGHT, m_drive);

    m_wait1 = new Wait(1);
    m_wait2 = new Wait(2);
    
    m_launch = new ParallelRaceGroup(
      new Wait(AutoConstants.kLAUNCH_TIMER),
      new ShootPowerCells(m_shooter, AutoConstants.kLAUNCH_LOWER_VOLTAGE, AutoConstants.kLAUNCH_UPPER_VOLTAGE),
      new SequentialCommandGroup(new Wait(1), new ShootHopperFeed(m_hopper))
      );

    m_deployHarvester = new DeployHarvester(m_hopper, m_intake);

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
        m_drive2 = m_lDrive2;
        break;
      case 1:
        m_drive1 = m_mDrive1;
        m_drive2 = m_mDrive2;
        break;
      case 2:
        m_drive1 = m_rDrive1;
        m_drive2 = m_rDrive2;
        break;
      default:
        m_drive1 = m_mDrive1;
        m_drive2 = m_mDrive2;
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

    if (m_currentCommand == m_drive1) {
      return startWait();
    }

    if (m_currentCommand == m_wait2
        || m_currentCommand == m_wait1) {
      if (m_waitCount >= m_console.getROT_SW_1()) {
        return startLaunch();
      } else {
        dashboardCmd("Wait1+");
        switchCommand(m_wait1);
        m_waitCount++;
        return false;
      }
    }

    if (m_currentCommand == m_launch) {
      // insert drive two command here
      return startDrive2();
    }

    if (m_currentCommand == m_drive2) {
      // insert deploy harvester command here
      return startDeploy();
    }

    if (m_currentCommand == m_deployHarvester) {
      return true;
    }

    return true;
  }

  private void switchCommand(final Command cmd) {
    m_currentCommand.end(false);
    m_currentCommand = cmd;
    m_currentCommand.initialize();
  }
  
  private boolean startWait() {
    boolean isFinished = true;
    if (m_console.getROT_SW_1() > 0) {
      dashboardCmd("Wait2");
      switchCommand(m_wait2);
      m_waitCount = 1;
      isFinished = false;
    } else {
      isFinished = startLaunch();
    }
    return isFinished;
  }

  private boolean startLaunch() {
    boolean isFinished = true;
    if (m_console.cnsl_btn_2.get()) {
      dashboardCmd("Launch 0");
      switchCommand(m_launch);
      isFinished = false;
    } else {
      isFinished = startDrive2();
    }
    return isFinished;
  }

  private boolean startDrive2() {
    boolean isFinished = true;
    if (m_console.cnsl_btn_3.get()) {
      dashboardCmd("Drive 2");
      switchCommand(m_drive2);
      isFinished = false;
    } else {
      isFinished = startDeploy();
    }
    return isFinished;
  }

  private boolean startDeploy() {
    boolean isFinished = true;
    if (m_console.cnsl_btn_4.get()) {
      dashboardCmd("Deploy");
      switchCommand(m_deployHarvester);
      isFinished = false;
    } else {
      isFinished = true;
    }
    return isFinished;
  }

}
