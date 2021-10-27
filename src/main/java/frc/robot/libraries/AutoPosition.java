// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.libraries;

/** Add your docs here. */
public enum AutoPosition {
   
    LEFT(0),
    MIDDLE(1),
    RIGHT(2)
    ;
    
    private int m_iPosSWSetting;

    AutoPosition(int posSWSetting) {
        this.m_iPosSWSetting = posSWSetting;
    }

    public int getPosSWSetting() {
        return m_iPosSWSetting;
    }

    public static AutoPosition getByIndex(int posIndex) {
        for (AutoPosition autoPos : AutoPosition.values()) {
            if (autoPos.m_iPosSWSetting == posIndex) {
                return autoPos;
            }
        }
        return null;
    }

}
