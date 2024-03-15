// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix.led.ColorFlowAnimation;
import com.ctre.phoenix.led.RainbowAnimation;
import com.ctre.phoenix.led.SingleFadeAnimation;
import com.ctre.phoenix.led.CANdle.LEDStripType;

import edu.wpi.first.hal.PowerJNI;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
//import frc.robot.Constants;
import frc.robot.Constants.LED;
import frc.robot.Robot;

public class LEDSubsystem extends SubsystemBase {
  CANdle candle = new CANdle(LED.CANdleID);


  ColorFlowAnimation redFlowAnimation;
  RainbowAnimation rainbowAnim;
  SingleFadeAnimation redFadeAnim;

  int numLED = LED.numLED;

  int baLEDLeft = 0; //Boot Animation LED Left
  int baLEDRight = numLED; //Boot ANimated LED Right
  boolean baCalledRedFade = false;
  double baLEDR = LED.teamR;
  double baLEDB = LED.teamB;
  double baLEDG = LED.teamG;
  

  /*  Info about LED animations
   * 
   *  See https://team2168.org/javadoc/com/ctre/phoenix/led/package-tree.html
   * 
   * Larson - A set of LEDs that bounces from one side to another
   * Twinkle - Twinkles animations on and off
   * Fire - FLAMES!!!! YEAH!!! EXPLOSIONS!!!!!!! WOOO!!!!!
   * Strobe - Its a strobe, likely don't use
   * RGB Fade - Fades ALL LEDs between R, G, and B.
   * 
   * 
   */



  public LEDSubsystem() {
    CANdleConfiguration config = new CANdleConfiguration(); //Configs
    config.stripType = LEDStripType.RGB; //Who the heck likes RBG?
    config.brightnessScalar = 0.5; 

    candle.configAllSettings(config);


    redFlowAnimation = new ColorFlowAnimation(LED.teamR, LED.teamG, LED.teamB); //Guess it does a flow
    redFlowAnimation.setNumLed(numLED);
    redFlowAnimation.setSpeed(0.5);

    rainbowAnim = new RainbowAnimation(); //Makes the robot based 
    rainbowAnim.setSpeed(0.05);
    rainbowAnim.setNumLed(numLED);

    redFadeAnim = new SingleFadeAnimation(LED.teamR, LED.teamG, LED.teamB);
    redFadeAnim.setSpeed(0.5);
    redFadeAnim.setNumLed(numLED);
    



  }

  public void animRedFire()
  {
    candle.clearAnimation(0);
    candle.animate(redFlowAnimation);
  }
  public void animRedFade()
  {
    candle.clearAnimation(0);
    candle.animate(redFadeAnim);

  }

  public void setLEDs(int r, int g, int b) //kinda redundant but good if we need some additional settings
  {
    candle.clearAnimation(0);
    candle.setLEDs(r, g, b);

  }

  public void setRed() //it cool B)
  {
    setLEDs(255,0,0);
  }
  public void setBlue() //Likely when we disable (like a BSOD)
  {
    setLEDs(32,0,191);
  }
  public void setGreen() //Idk when we want green I guess??
  {
    setLEDs(0,255,0);
  }
  public void setOrange() //likely to be used for when we have a piece in intake
  {
    setLEDs(255,149,65);
  }


  public void setTeam() //When we want to be our team color
  {
    setLEDs(LED.teamR, LED.teamG, LED.teamB);
  }


  @Override
  public void periodic() {
    double batteryVoltage = PowerJNI.getVinVoltage();
    if(batteryVoltage < 9) //That one thing that sees the drop in battery voltage and stops LEDs if low uwu
    {
      setLEDs(0, 0, 0);
    }

    if(Robot.runBootAnimation)
    {
      if(baLEDLeft < baLEDRight)
      {
        candle.setLEDs((int)baLEDR, (int)baLEDG, (int)baLEDB, 0, baLEDLeft, 1);
        candle.setLEDs((int)baLEDR, (int)baLEDG, (int)baLEDB, 0, baLEDRight, 1);
        baLEDLeft += 0.05;
        baLEDRight -= 0.05;
      }
      else
      {
        if(!baCalledRedFade)
        {
          
          
          baLEDR = MathUtil.interpolate(baLEDR, 0, 0.05); //dimming time baby
          baLEDG = MathUtil.interpolate(baLEDG, 0, 0.05);
          baLEDB = MathUtil.interpolate(baLEDB, 0, 0.05);

          baLEDR = MathUtil.clamp(baLEDR, 0, 255); //clamping time baby
          baLEDB = MathUtil.clamp(baLEDB, 0, 255);
          baLEDG = MathUtil.clamp(baLEDG, 0, 255);

          candle.setLEDs((int)baLEDR, (int)baLEDG, (int)baLEDB);
          
          if(baLEDR+baLEDG+baLEDB <= 1) //Haha look at me doing something partially smart
          {
            animRedFade();
            baCalledRedFade = true;
          }
        }
      }
    } 


  }

}


