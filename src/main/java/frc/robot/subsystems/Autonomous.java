/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import java.io.File;
import java.util.function.IntSupplier;

import javax.xml.parsers.DocumentBuilder;
import javax.xml.parsers.DocumentBuilderFactory;
import javax.xml.xpath.XPath;
import javax.xml.xpath.XPathConstants;
import javax.xml.xpath.XPathFactory;

import org.w3c.dom.Document;
import org.w3c.dom.Element;
import org.w3c.dom.Node;
import org.w3c.dom.NodeList;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Autonomous extends SubsystemBase {
  
  private Document m_patternAndCommandDoc;
  private XPath m_xPath;
  private final String m_strPatternxmlFilename = "/home/lvuser/deploy/Autonomous.xml";
  private String m_strPattern;

  private Node m_node;
  private NodeList m_nlCommands;
  private int m_nlCommandsLength = 0;
  private int m_nlCommandNextIndex = 0;

  private Element m_element;

  private IntSupplier m_pov1;
  private IntSupplier m_pov2;
  /**
   * Creates a new Autonomous.
   */
  public Autonomous() {

  }

  public void getXML(IntSupplier pov1, IntSupplier pov2){
    // load xml
    m_pov1 = pov1;
    m_pov2 = pov2;
    try {
      File patternAndCommandFile = new File(m_strPatternxmlFilename);
      DocumentBuilderFactory dbFactory = DocumentBuilderFactory.newInstance();
      DocumentBuilder dBuilder;

      dBuilder = dbFactory.newDocumentBuilder();

      m_patternAndCommandDoc = dBuilder.parse(patternAndCommandFile);
      m_patternAndCommandDoc.getDocumentElement().normalize();
      m_xPath = XPathFactory.newInstance().newXPath();
      searchAutoXml();
    } catch (Exception e){
      e.printStackTrace();
    }
    

    SmartDashboard.putBoolean("XML loaded", true);
    //System.out.println("hi");
  }
  //Do confusing stuff. So confuse, I not even sure if spelled confusing right?
  public void searchAutoXml() {
    //String strStartingPosition = Robot.m_oi.getAutoStartPos();
    String strStartingPosition = getAutoStartPos();
    String strPattern = getAutoPatern();
  
    try {

      String searchExpr = "//startingPosition[@name=\"" + strStartingPosition + "\"]//pattern[@name=\"" + strPattern + "\"]/text()";
     // String searchExpr = "//startingPosition[@name=\"" + strStartingPosition + "\"]//targetPosition[@name=\"" + strTargetPosition + "\"]/text()";

      SmartDashboard.putString("searchexpr", searchExpr);

      NodeList nodeList = (NodeList) m_xPath.compile(searchExpr).evaluate(m_patternAndCommandDoc, XPathConstants.NODESET);
      
      if (nodeList.getLength() == 1){
        m_strPattern = nodeList.item(0).getNodeValue();
        SmartDashboard.putString("Pattern", m_strPattern);
      } else {
        m_strPattern = "crossAuto";
        SmartDashboard.putString("Pattern", "no Pattern found");
      }

      searchExpr = "//patternCommands[@name=\"" + m_strPattern + "\"]";

      SmartDashboard.putString("searchexpr2", searchExpr);

      nodeList = (NodeList) m_xPath.compile(searchExpr).evaluate(m_patternAndCommandDoc, XPathConstants.NODESET);
      
      if (nodeList.getLength() == 1){
        m_node = nodeList.item(0);
        m_nlCommands = m_node.getChildNodes();
        m_nlCommandsLength = m_nlCommands.getLength();
        SmartDashboard.putNumber("numCommandsFound", m_nlCommandsLength);
      } else {
        m_nlCommandsLength = 0;
        SmartDashboard.putString("Pattern", "no Command found");
      }

    } catch (Exception e) {
      e.printStackTrace();
    }
  }

  public String getAutoStartPos(){
    int startPosNumber = m_pov1.getAsInt();
    String startPosValue = "1";
    switch(startPosNumber){
      case 0:
        startPosValue = "1";
        break;
      case 45:
        startPosValue = "2";
        break;
      case 90:
        startPosValue = "3";
        break;
      case 135:
        startPosValue = "4";
        break;
      case 180:
        startPosValue = "5";
        break;  
      case 225:
        startPosValue = "6";
        break;      
      default:
        startPosValue = "6";
    }
    return startPosValue;
  }

  public String getAutoPatern(){
    int AutoParNumber = m_pov2.getAsInt();
    String ParValue = "1";
    switch(AutoParNumber){
      case 0:
        ParValue = "1";
        break;
      case 45:
        ParValue = "2";
        break;
      case 90:
        ParValue = "3";
        break;
      case 135:
        ParValue = "4";
        break;
      case 180:
        ParValue = "5";
        break;
      case 225:
        ParValue = "6";
        break;    
      default:
        ParValue = "6";
    }
    return ParValue;
  }


  public String getNextCmd() { // !! very high in fat !!
    String command = "";
    Node node;

    for (int i=m_nlCommandNextIndex; i < m_nlCommandsLength; i++){
      node = m_nlCommands.item(i);
      if (node.getNodeType() == Node.ELEMENT_NODE) {
        m_element = (Element) node;
        command = m_element.getAttribute("name");
        m_nlCommandNextIndex = i + 1;
        break;
      }
    }

    return command;
  }

  public boolean isFinished(){
    return true;
  }

  public double getDoubleCommandValue(){
    return Double.valueOf(m_element.getTextContent());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
