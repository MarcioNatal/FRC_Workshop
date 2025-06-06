// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LedControl extends SubsystemBase 
{
  private AddressableLED m_led;
  private AddressableLEDBuffer m_ledBuffer;
  // Store what the last hue of the first pixel is
  private int m_rainbowFirstPixelHue;
  /** Creates a new LedControl. */
  private boolean ledsOn = true;
  public LedControl() 
  {

    //Blue and Green are inverted

    // PWM port 9
    // Must be a PWM header, not MXP or DIO
    m_led = new AddressableLED(0);

    // Reuse buffer
    // Default to a length of 60, start empty output
    // Length is expensive to set, so only set it once, then just update data
    m_ledBuffer = new AddressableLEDBuffer(60);
    m_led.setLength(m_ledBuffer.getLength());

    // Set the data
    m_led.setData(m_ledBuffer);
    m_led.start();

  }

  @Override
  public void periodic() 
  {
    // This method will be called once per scheduler run

    if (ledsOn)
    {
      setRainbow();
      ledsOn = false;
    }
    
    
  }

  // Sets the LED color to red
  public void setRedLed()
  {

    for (var i = 0; i < m_ledBuffer.getLength(); i++) 
    {
      // Sets the specified LED to the RGB values for red
      // green e blue invertidos
      m_ledBuffer.setRGB(i, 255, 0, 0);
    }
    m_led.setData(m_ledBuffer);
    
  
  
  }
 
  public void setOrangeLed()
  {

    for (var i = 0; i < m_ledBuffer.getLength(); i++) 
    {
      // Sets the specified LED to the RGB values for red
      // green e blue invertidos
      m_ledBuffer.setRGB(i, 255, 0, 165);

    }
    m_led.setData(m_ledBuffer);
    
  
  
  }
  // Sets the LED color to blue 
  public void setBlueLed()
  { 
    for (var i = 0; i < m_ledBuffer.getLength(); i++) 
    {
      // Sets the specified LED to the HSV values for red
      m_ledBuffer.setRGB(i,0,255,0);
    }
   m_led.setData(m_ledBuffer);
  
  } 
  // Sets the LED color to green
  public void setGreenLed()
  {
    for (var i = 0; i < m_ledBuffer.getLength(); i++) 
    {
      m_ledBuffer.setRGB(i, 0, 0, 255);
    }
   m_led.setData(m_ledBuffer);
  
  } 

  public void setPinkLed()
  {
    for (var i = 0; i < m_ledBuffer.getLength(); i++) 
    {
      m_ledBuffer.setRGB(i, 255, 203, 192);
    }
   m_led.setData(m_ledBuffer);
  } 
  
  // Sets the LED color to yellow
  public void setYellowLed()
  {
    for (var i = 0; i < m_ledBuffer.getLength(); i++) 
    {
      // Sets the specified LED to the HSV values for red
      m_ledBuffer.setRGB(i, 255, 0, 255);
    }
   m_led.setData(m_ledBuffer);
  
  } 
  // Sets the LED color to purple 
  public void setPurpleLed()
  {
    for (var i = 0; i < m_ledBuffer.getLength(); i++) 
    {
      // Sets the specified LED to the HSV values for red
      m_ledBuffer.setRGB(i, 128, 128, 0);
    }
   m_led.setData(m_ledBuffer);
  
  }
  // Sets the LED color to white
  public void setWhiteLed()
  {
    for (var i = 0; i < m_ledBuffer.getLength(); i++) 
    {
      m_ledBuffer.setRGB(i, 255, 255, 255);
    }
   m_led.setData(m_ledBuffer);
  
  }
  // Sets the LED color to black  
  public void setBlackLed()
  {
    for (var i = 0; i < m_ledBuffer.getLength(); i++) 
    {
      // Sets the specified LED to the HSV values for red
      m_ledBuffer.setRGB(i, 0, 0, 0);
    }
   m_led.setData(m_ledBuffer);
  
  }

  
  public void setRainbow ()
  {
    // For every pixel
    for (var i = 0; i < m_ledBuffer.getLength(); i++) 
    {
      // Calculate the hue - hue is easier for rainbows because the color
      // shape is a circle so only one value needs to precess
      final var hue = (m_rainbowFirstPixelHue + (i * 180 / m_ledBuffer.getLength())) % 180;
      // Set the value
      m_ledBuffer.setHSV(i, hue, 255, 128);
    }
    // Increase by to make the rainbow "move"
    m_rainbowFirstPixelHue += 3;
    // Check bounds
    m_rainbowFirstPixelHue %= 180;

    m_led.setData(m_ledBuffer);
  }

}
