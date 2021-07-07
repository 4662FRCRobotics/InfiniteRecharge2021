// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.libraries;

import frc.robot.Constants.ConsoleCommandConstants;
import edu.wpi.first.wpilibj2.command.Command;


// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


//import static edu.wpi.first.wpilibj.util.ErrorMessages.requireNonNullParam;

import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.concurrent.atomic.AtomicInteger;
import java.util.concurrent.locks.ReentrantLock;
import java.util.function.IntSupplier;

/**
 * The {@link SendableChooser} class is a useful tool for presenting a selection of options to the
 * {@link SmartDashboard}.
 *
 * <p>For instance, you may wish to be able to select between multiple autonomous modes. You can do
 * this by putting every possible Command you want to run as an autonomous into a {@link
 * SendableChooser} and then put it into the {@link SmartDashboard} to have a list of options appear
 * on the laptop. Once autonomous starts, simply ask the {@link SendableChooser} what the selected
 * value is.
 *
 * @param <V> The type of the values to be stored
 */
public class ConsoleCommand {

  /** The key for the default value. */
  private static final String DEFAULT = "default";
  /** The key for the selected option. */
  private static final String SELECTED = "selected";
  /** The key for the active option. */
  private static final String ACTIVE = "active";
  /** The key for the option array. */
  private static final String OPTIONS = "options";
  /** The key for the instance number. */
  private static final String INSTANCE = ".instance";
  /** A map linking strings to the objects the represent. */
  private final Map<String, Command> m_map = new LinkedHashMap<>();

  private String m_defaultChoice = "";
  
 // private final int m_instance;
  private static final AtomicInteger s_instances = new AtomicInteger();
  
  /** Creates a new ConsoleCommand. */
  public ConsoleCommand() {

  }

  /**
   * Adds the given object to the list of options. On the {@link SmartDashboard} on the desktop, the
   * object will appear as the given name.
   *
   * @param name the name of the option
   * @param object the option
   */
  public void addOption(String name, Command command) {
    m_map.put(name, command);
  }

  /*
  * given the position and pattern settings find a matching name
  * if the settings are out of bounds use the default
  * and hope someone defined a default command
  */
  public String getPatternName(IntSupplier position, IntSupplier pattern) {

    int iPosition = position.getAsInt() / ConsoleCommandConstants.kPOV_INTERVAL;
    int iPattern = pattern.getAsInt() / ConsoleCommandConstants.kPOV_INTERVAL;
    String patternName = ConsoleCommandConstants.kDEFAULT_PATTERN_NAME;

    if (iPosition <= ConsoleCommandConstants.kPOS_PATTERN_NAME.length) {
      if (iPattern <= ConsoleCommandConstants.kPOS_PATTERN_NAME[iPosition].length) {
        patternName = ConsoleCommandConstants.kPOS_PATTERN_NAME[iPosition][iPattern];
      }
    }
    
    return patternName;
  }

  /**
   * Returns the selected option. If there is none selected, it will return the default. If there is
   * none selected and no default, then it will return {@code null}.
   *
   * @return the option selected
   */
  public Command getSelected(IntSupplier position, IntSupplier pattern) {

    String sPatternName = getPatternName(position, pattern);
    return m_map.get(sPatternName);

  }

}