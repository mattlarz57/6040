package org.firstinspires.ftc.teamcode.GhostExamples;


import java.util.ArrayList;
import java.util.Arrays;

public class ButtonValues extends ControllerValues<Boolean> {
  public final static String buttonA="a";
  public final static String buttonB="b";
  public final static String buttonX="x";
  public final static String buttonY="y";
  public final static String dpadUp="du";
  public final static String dpadDown="dd";
  public final static String dpadLeft="dl";
  public final static String dpadRight="dr";
  public final static String bumperRight = "br";
  public final static String bumperLeft = "bl";

 
   
  
  public ButtonValues() {
    super(false,
          new ArrayList<Boolean>(Arrays.asList(new Boolean[]{false,false,false,false,false,false,false,false,false,false})),
          new ArrayList<String>(Arrays.asList(new String[]{buttonA,buttonB,buttonX,buttonY,dpadUp,dpadDown,dpadLeft,dpadRight,bumperRight,bumperLeft}))
         ); 
    
  }
}
