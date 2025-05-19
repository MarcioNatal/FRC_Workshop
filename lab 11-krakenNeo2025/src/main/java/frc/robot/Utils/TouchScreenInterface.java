// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Utils;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TouchScreenInterface extends SubsystemBase {


  public enum VirtualButton {
    /** A button. */
    kA("A"),
    /** B button. */
    kB("B"),
    /** C button. */
    kC("C"),
    /** D button. */
    kD("D"),
    /** E button. */
    kE("E"),
    /** F button. */
    kF("F"),
    /** G button. */
    kG("G"),
    /** H button. */
    kH("H"),
    /** I button. */
    kI("I"),
    /** J button. */
    kJ("J"),
    /** K button. */
    kK("K"),
    /** L button. */
    kL("L"),
    /** L button. */
    kL1("L1"),
    /** L button. */
    kL2("L2"),
    /** L button. */
    kL3("L3"),
    /** L button. */
    kL4("L4");

    public final String value;

    VirtualButton(String value) {
      this.value = value;
    }

    /**
     * Get the human-friendly name of the button, matching the relevant methods. This is done by
     * stripping the leading `k`, and appending `VirtualButton`.
     *
     * <p>Primarily used for automated unit tests.
     *
     * @return the human-friendly name of the button.
     */
    @Override
    public String toString() {
      // Remove leading `k`
      return this.name().substring(1) + "Button";
    }
  }

  // Instance NetworkTables and Entries 

  //NetworkTableInstance inst = NetworkTableInstance.getDefault();
  //BooleanTopic booleanTopic[] = new BooleanTopic[16];
  boolean[] btnValue = new boolean[16];
  //BooleanEntry[] bEntry = new BooleanEntry[16];
  //NetworkButton[] networkButtons = new NetworkButton[16];

  boolean enableInterface = false;


  /** Creates a new TouchScreenInterface. */
  public TouchScreenInterface() {
    for (VirtualButton button : VirtualButton.values()) {
      // Create a new boolean topic for each button
      //booleanTopic[button.ordinal()] = new BooleanTopic(inst.getBooleanTopic(button.toString()));
      // Set the default value of the button to false
      btnValue[button.ordinal()] = false;
      // Create a new boolean entry for each button
      //bEntry[button.ordinal()] = booleanTopic[button.ordinal()].getEntryEx("boolean", btnValue[button.ordinal()]);
      // Create a new network button for each button
      //networkButtons[button.ordinal()] = new NetworkButton(bEntry[button.ordinal()].getTopic()
      //                                        .subscribeEx("boolean", btnValue[button.ordinal()]));
      // Set the default value of the button to false and put it on the SmartDashboard
      SmartDashboard.putBoolean(button.toString()+"Value", btnValue[button.ordinal()]);
    }
    // Put the enableInterface boolean on the SmartDashboard
    SmartDashboard.putBoolean("enableInterface", enableInterface);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    // Get the value of the enableInterface boolean from the SmartDashboard
    enableInterface = SmartDashboard.putBoolean("enableInterface", isInterfaceEnabled());
    for (VirtualButton button : VirtualButton.values()) {
      // Get the value of the button from the SmartDashboard
      btnValue[button.ordinal()] = SmartDashboard.getBoolean(button.toString()+"Value", btnValue[button.ordinal()]);
      // Set the value of the button to the value from the SmartDashboard
      //bEntry[button.ordinal()].get(btnValue[button.ordinal()]);
      //bEntry[button.ordinal()].set(btnValue[button.ordinal()]);
    }

  }

  // Get the boolean value of the button
  public boolean getButtonValue(VirtualButton button) {
    return btnValue[button.ordinal()];
  }

  // Get the NetworkButton object of the button
  /* 
  public NetworkButton getButton(VirtualButton button) {
    return networkButtons[button.ordinal()];
  }
  //*/

  // Enable or disable the interface
  public void enableInterface() {
    enableInterface = true;
  }

  // Enable or disable the interface
  public void disableInterface() {
    enableInterface = false;
  }

  // Get the value of the enableInterface boolean
  public boolean isInterfaceEnabled() {
    return enableInterface;
  }

}
