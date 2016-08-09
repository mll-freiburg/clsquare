/*
libJaco - Kinova Jaco interface
Copyright (c) 2010-2012 Machine Learning Lab, 
Thomas Lampe

All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are
met:

   * Redistributions of source code must retain the above copyright
     notice, this list of conditions and the following disclaimer.
   * Redistributions in binary form must reproduce the above copyright
     notice, this list of conditions and the following disclaimer in
     the documentation and/or other materials provided with the
     distribution.
   * Neither the name of the <ORGANIZATION> nor the names of its
     contributors may be used to endorse or promote products derived
     from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
DAMAGE. 
*/

using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.IO;
using Kinova.API.Jaco;
using Kinova.API.Jaco.Diagnostic;
using Kinova.API.Jaco.ValidationTools.Exceptions;
using Kinova.DLL.SafeGate;
using Kinova.DLL.Data.Jaco;
using Kinova.DLL.Data.Util;
using Kinova.DLL.Data.Jaco.Control;
using Kinova.DLL.Data.Jaco.Config;
using Kinova.DLL.Data.Jaco.Diagnostic;
using Kinova.API.Jaco.Configurations;

namespace JacoInterface
{

class Jaco
{
  static CJacoArm _jaco;
  static CZoneList _zones;
  static CJoystickValue _cmd;
  static float[] _data;
  static bool _api = false;
  static bool _cartesian = true;
  static string _error = "Error: ";
  static string _info = "";
  static string _warning = "Warning: ";

  static void Main (string[] args)
  {
    bool dll = args.Length > 0 && String.Compare(args[0], "dll") == 0;
    if (dll) {
      _error   = "#ERROR [JACO]: ";
      _info    = "#INFO  [JACO]: ";
      _warning = "#WARNING [JACO]: ";
    }

    try {
      _jaco = new CJacoArm(Crypto.GetInstance().Encrypt("C6H12O6h2so4"));
      Console.WriteLine(_info + "Successfully connected to Jaco.");
    } catch (CAccessDeniedException) {
      Console.WriteLine(_error + "Access to Jaco denied. Please check password and USB connection.");
      return;
    } catch (Exception) {
      Console.WriteLine(_error + "Could not connect to Jaco.");
      return;
    }

    _cmd = new CJoystickValue();
    _zones = _jaco.ConfigurationsManager.GetProtectionZones();
    _jaco.ControlManager.StartControlAPI();
    _api = true;
    _data = new float[37];
    for (int i=0; i<37; i++) _data[i] = 0f;

    if (dll) {
      Console.WriteLine(_info + "Active protection zones: " + _zones.NbZoneActive);
      return;
    }

    TerminalCommand(args);
    Deinit();
  }

  static void Test ()
  {
  ///< \todo time

    CJacoGripper g = _jaco.ConfigurationsManager.GetJacoGripperInfo();
    CJacoFinger f1 = g.Fingers[CJacoGripper.FINGER_1];
    Console.WriteLine("Pos: " + f1.ActualPosition);
    Console.WriteLine("Current: " + f1.ActualCurrent);
    Console.WriteLine("AvgCur: " + f1.ActualAverageCurrent);
    Console.WriteLine("Force: " + f1.ActualForce);
    Console.WriteLine("Force: " + f1.MaxForce);
    Console.WriteLine("Speed: " + f1.ActualSpeed);
    Console.WriteLine("Accel: " + f1.ActualAcceleration);
    Console.WriteLine("Temp: " + f1.ActualTemperature);
    Console.WriteLine();

    CCartesianInfo info = _jaco.ControlManager.GetForceCartesianInfo();
    System.Console.WriteLine("         Force X : " + info.X);
    System.Console.WriteLine("         Force Y : " + info.Y);
    System.Console.WriteLine("         Force Z : " + info.Z);
    System.Console.WriteLine("   Force Theta X : " + info.ThetaX);
    System.Console.WriteLine("   Force Theta Y : " + info.ThetaY);
    System.Console.WriteLine("   Force Theta Z : " + info.ThetaZ);
    System.Console.WriteLine(" Force Finger #1 : " + info.Finger1);
    System.Console.WriteLine(" Force Finger #2 : " + info.Finger2);
    System.Console.WriteLine(" Force Finger #3 : " + info.Finger3);
    Console.WriteLine();

    CAngularInfo ang = _jaco.ControlManager.GetCurrentAngularInfo();
    System.Console.WriteLine("   Current Joint 1 : " + ang.Joint1);
    System.Console.WriteLine("   Current Joint 2 : " + ang.Joint2);
    System.Console.WriteLine("   Current Joint 3 : " + ang.Joint3);
    System.Console.WriteLine("   Current Joint 4 : " + ang.Joint4);
    System.Console.WriteLine("   Current Joint 5 : " + ang.Joint5);
    System.Console.WriteLine("   Current Joint 6 : " + ang.Joint6);
    System.Console.WriteLine(" Current Finger #1 : " + ang.Finger1);
    System.Console.WriteLine(" Current Finger #2 : " + ang.Finger2);
    System.Console.WriteLine(" Current Finger #3 : " + ang.Finger3);
    Console.WriteLine();

    ang = _jaco.ControlManager.GetForceAngularInfo();
    System.Console.WriteLine("   Force Joint 1 : " + ang.Joint1);
    System.Console.WriteLine("   Force Joint 2 : " + ang.Joint2);
    System.Console.WriteLine("   Force Joint 3 : " + ang.Joint3);
    System.Console.WriteLine("   Force Joint 4 : " + ang.Joint4);
    System.Console.WriteLine("   Force Joint 5 : " + ang.Joint5);
    System.Console.WriteLine("   Force Joint 6 : " + ang.Joint6);
    System.Console.WriteLine(" Force Finger #1 : " + ang.Finger1);
    System.Console.WriteLine(" Force Finger #2 : " + ang.Finger2);
    System.Console.WriteLine(" Force Finger #3 : " + ang.Finger3);
    Console.WriteLine();

    GetData();
    for (int i=0; i<37; i++) Console.Write(_data[i]+"  ");
    Console.WriteLine();

    System.Console.WriteLine("Max Zones: " + CZoneList.NB_ZONE_MAX);
  }

  /*******************************/
  /** interface for C++ clients **/
  /*******************************/

  static void Deinit ()
  {
    Stop();
    _jaco.ControlManager.SetCartesianControl();
    _jaco.ControlManager.StopControlAPI();
    _jaco.CloseAll();
  }

  static void Stop ()
  {
    SetDirection(_cartesian, 0f, 0f, 0f, 0f, 0f, 0f);
  }

  static bool CheckControl ()
  {
    if (!_jaco.ControlManager.IsApiInControl()) {
      if (_api) {
        Console.WriteLine(_warning + "API has lost control of Jaco!");
        _api = false;
      }
      return false;
    }
    return true;
  }

  static void CheckMode (bool cartesian)
  {
    if (_cartesian == cartesian) return;

    if (cartesian)
      _jaco.ControlManager.SetCartesianControl();
    else {
      Console.WriteLine(_warning + "Jaco was set to angular mode. Improper use may damage the arm.");
      _jaco.ControlManager.SetAngularControl();
    }
    _cartesian = cartesian;
  }

  static bool SetDirection (bool cartesian, float d1, float d2, float d3, float d4, float d5, float d6)
  {
    if (!CheckControl()) return false;
    CheckMode(cartesian);

    _cmd.InclineLR = d1;
    _cmd.InclineFB = d2;
    _cmd.Rotate    = d3;
    _cmd.MoveFB    = d4;
    _cmd.MoveLR    = d5;
    _cmd.PushPull  = d6;
    _jaco.ControlManager.SendJoystickFunctionality(_cmd);
    return true;
  }

  static bool SetPosition (bool cartesian, float x1, float x2, float x3, float x4, float x5, float x6, float f1, float f2, float f3)
  {
    if (!CheckControl()) return false;
    CheckMode(cartesian);
    _cartesian = cartesian;

    CTrajectoryInfo _point = new CTrajectoryInfo();
    SetTargetPosition(_point, cartesian, x1, x2, x3, x4, x5, x6, f1, f2, f3);

    CPointsTrajectory trajectory = new CPointsTrajectory();
    trajectory.Add(_point);
    _jaco.ControlManager.SendTrajectoryFunctionnality(trajectory);
    return true;
  }

  static void SetTargetPosition (CTrajectoryInfo point, bool cartesian, float x1, float x2, float x3, float x4, float x5, float x6, float f1, float f2, float f3)
  {
    if (cartesian) {
      point.UserPosition.Position = new CVectorEuler();
      point.UserPosition.Position.Position[CVectorEuler.COORDINATE_X] = x1;
      point.UserPosition.Position.Position[CVectorEuler.COORDINATE_Y] = x2;
      point.UserPosition.Position.Position[CVectorEuler.COORDINATE_Z] = x3;
      point.UserPosition.Position.Rotation[CVectorEuler.THETA_X] = x4;
      point.UserPosition.Position.Rotation[CVectorEuler.THETA_Y] = x5;
      point.UserPosition.Position.Rotation[CVectorEuler.THETA_Z] = x6;
      point.UserPosition.PositionType = CJacoStructures.PositionType.CartesianPosition;
    } else {
      point.UserPosition.AnglesJoints = new CVectorAngle();
      point.UserPosition.AnglesJoints.Angle[CVectorAngle.JOINT_1] = x1;
      point.UserPosition.AnglesJoints.Angle[CVectorAngle.JOINT_2] = x2;
      point.UserPosition.AnglesJoints.Angle[CVectorAngle.JOINT_3] = x3;
      point.UserPosition.AnglesJoints.Angle[CVectorAngle.JOINT_4] = x4;
      point.UserPosition.AnglesJoints.Angle[CVectorAngle.JOINT_5] = x5;
      point.UserPosition.AnglesJoints.Angle[CVectorAngle.JOINT_6] = x6;
      point.UserPosition.PositionType = CJacoStructures.PositionType.AngularPosition;
    }
    point.UserPosition.HandMode = CJacoStructures.HandMode.PositionMode;
    point.UserPosition.FingerPosition[0] = f1;
    point.UserPosition.FingerPosition[1] = f2;
    point.UserPosition.FingerPosition[2] = f3;
  }

  static bool SetPositionWithSpeed (bool cartesian,  float x1,  float x2,  float x3,  float x4,  float x5,  float x6,  float f1,  float f2,  float f3, float sc, float sa, float sf)
  {
    if (!CheckControl()) return false;
    CheckMode(cartesian);
    _cartesian = cartesian;

    CTrajectoryInfo _point = new CTrajectoryInfo();
    SetTargetPosition(_point, cartesian, x1, x2, x3, x4, x5, x6, f1, f2, f3);
    SetTargetSpeed(_point, sc, sa, sf);

    CPointsTrajectory trajectory = new CPointsTrajectory();
    trajectory.Add(_point);
    _jaco.ControlManager.SendTrajectoryFunctionnality(trajectory);
    return true;
  }

  static void SetTargetSpeed (CTrajectoryInfo point, float linear, float angular, float finger)
  {
    if (linear >= 0f && linear <= 0.15f) {
      point.ZoneLimitation.LinearSpeed = linear;
      Console.WriteLine(_info + "Linear speed set to " + linear);
      point.LimitationActive = true;
    }
    if (angular >= 0f && angular <= 0.6f) {
      point.ZoneLimitation.AngularSpeed = angular;
      point.LimitationActive = true;
    }
    if (finger >= 0f && finger <= 0.15f) {
      point.ZoneLimitation.FingersSpeed = finger;
      point.LimitationActive = true;
    }
  }

  static bool SetSpeed (bool cartesian, float speed)
  {
    if (!CheckControl()) return false;
    CClientConfigurations c = _jaco.ConfigurationsManager.GetClientConfigurations();
    if (speed < 0f) speed = 0f;
    if (cartesian) {
      if (speed > 0.15f) speed = 0.15f;
      c.MaxLinearSpeed = speed;
    } else {
      if (speed > 0.6f) speed = 0.6f;
      c.MaxAngularSpeed = speed;
    }
    _jaco.ConfigurationsManager.SetClientConfigurations(c);
    return true;
  }

  static float[] GetData ()
  {
    CVectorEuler   cpos   = _jaco.ConfigurationsManager.GetHandPosition();
    CVectorAngle   apos   = _jaco.ConfigurationsManager.GetJointPositions();
    CCartesianInfo fpos   = _jaco.ControlManager.GetPositioningCartesianInfo();
    CCartesianInfo cforce = _jaco.ControlManager.GetForceCartesianInfo();
    CAngularInfo   aforce = _jaco.ControlManager.GetForceAngularInfo();
    CAngularInfo   curr   = _jaco.ControlManager.GetCurrentAngularInfo();

    // cartesian position
    _data[0]  = cpos.Position[CVectorEuler.COORDINATE_X];
    _data[1]  = cpos.Position[CVectorEuler.COORDINATE_Y];
    _data[2]  = cpos.Position[CVectorEuler.COORDINATE_Z];
    _data[3]  = cpos.Rotation[CVectorEuler.THETA_X];
    _data[4]  = cpos.Rotation[CVectorEuler.THETA_Y];
    _data[5]  = cpos.Rotation[CVectorEuler.THETA_Z];

    // angular position
    _data[6]  = apos.Angle[CVectorAngle.JOINT_1];
    _data[7]  = apos.Angle[CVectorAngle.JOINT_2];
    _data[8]  = apos.Angle[CVectorAngle.JOINT_3];
    _data[9]  = apos.Angle[CVectorAngle.JOINT_4];
    _data[10] = apos.Angle[CVectorAngle.JOINT_5];
    _data[11] = apos.Angle[CVectorAngle.JOINT_6];

    // fingers
    _data[12] = fpos.Finger1;
    _data[13] = fpos.Finger2;
    _data[14] = fpos.Finger3;

    // API control
    _data[15] = _jaco.ControlManager.IsApiInControl() ? 1f : 0f;

    // cartesian force
    _data[16] = cforce.X;
    _data[17] = cforce.Y;
    _data[18] = cforce.Z;
    _data[19] = cforce.ThetaX;
    _data[20] = cforce.ThetaY;
    _data[21] = cforce.ThetaZ;

    // angular force
    _data[22] = aforce.Joint1;
    _data[23] = aforce.Joint2;
    _data[24] = aforce.Joint3;
    _data[25] = aforce.Joint4;
    _data[26] = aforce.Joint5;
    _data[27] = aforce.Joint6;

    // motor currents
    _data[28] = curr.Joint1;
    _data[29] = curr.Joint2;
    _data[30] = curr.Joint3;
    _data[31] = curr.Joint4;
    _data[32] = curr.Joint5;
    _data[33] = curr.Joint6;
    _data[34] = curr.Finger1;
    _data[35] = curr.Finger2;
    _data[36] = curr.Finger3;

    return _data;
  }

  static bool SetFingers (float f1, float f2, float f3)
  {
    Stop();
    GetData();

    // use whichever mode was used before to avoid slow switches
    return _cartesian ?
      SetPosition(_cartesian, _data[0], _data[1], _data[2], _data[3], _data[4],  _data[5],  f1, f2, f3) :
      SetPosition(_cartesian, _data[6], _data[7], _data[8], _data[9], _data[10], _data[11], f1, f2, f3);
  }

  static bool SetGripper (float tar)
  {
    return SetFingers(tar, tar, tar);
  }

  static bool CloseGripper ()
  {
    return SetGripper(45f);
  }

  static bool OpenGripper ()
  {
    return SetGripper(0f);
  }

  static bool GetControl ()
  {
    _jaco.ControlManager.StartControlAPI();
    return true;
  }

  /****************************/
  /** command-line utilities **/
  /****************************/

  static void PrintUsage ()
  {
    Console.WriteLine(_info + "Usage: mono " + System.AppDomain.CurrentDomain.FriendlyName + " <operation>");
    Console.WriteLine(_info + "Possible operations:");
    Console.WriteLine(_info + "    read:             continually read data from robot");
    Console.WriteLine(_info + "    control:          control robot via console (type \"help\" in command mode for listing)");
    Console.WriteLine(_info + "    zone:             lists the currently active protection zones");
    Console.WriteLine(_info + "    zone set <name>:  reads a list of protection zones from file <name> and sends them to the robot");
    Console.WriteLine(_info + "    zone get <name>:  gets the currently active protection zones from the robot and writes them to the file <name>");
    Console.WriteLine(_info + "    zone save <name>: serializes the protection files stored on the robot under the profile <name>");
    Console.WriteLine(_info + "    zone load <name>: deserializes the protection files in the profile <name>");
  }

  static void TerminalCommand (string[] args)
  {
    if (args.Length > 0 && String.Compare(args[0], "read") == 0) {
      KeepReading(true);
      return;
    }

    if (args.Length > 0 && String.Compare(args[0], "zone") == 0) {
      ZoneCommand(args);
      return;
    }

    if (args.Length > 0 && String.Compare(args[0], "control") == 0) {
      PositionControl();
      return;
    }

    if (args.Length > 0 && String.Compare(args[0], "test") == 0) {
      Test();
      return;
    }

    PrintUsage();
  }

  static void KeepReading (bool stop)
  {
    if (stop) Stop();
    while (true) {
      GetData();
      for (int i=0; i<15; i++) Console.Write(_data[i]+"  ");
      Console.WriteLine();
    }
  }

  static bool ZoneCommand (string[] args)
  {
    if (args.Length == 1) {
      PrintZones();
      return true;
    }

    if (args.Length > 2) {
      if (String.Compare(args[1], "save") == 0)
        return SaveZonesToProfile(args[2]);
      if (String.Compare(args[1], "load") == 0)
        return LoadZonesFromProfile(args[2]);
      if (String.Compare(args[1], "get") == 0)
        return SaveZonesToFile(args[2]);
      if (String.Compare(args[1], "set") == 0)
        return LoadZonesFromFile(args[2]);
    }

    PrintUsage();
    return false;
  }

  static bool PositionControl ()
  {
    Console.WriteLine("Jaco is in console control mode. Type \"help\" to get a list of available commands.");
    while (true) {
      string line = Console.ReadLine();
      if (String.Compare(line, "exit") == 0) break;
      if (String.Compare(line, "quit") == 0) break;
      if (String.Compare(line, "help") == 0) {
        Console.WriteLine("Available commands:");
        Console.WriteLine("  exit/quit:   exit program");
        Console.WriteLine("  stop:        stop all motors");
        Console.WriteLine("  read:        get current position");
        Console.WriteLine("  reclaim:     have API reclaim control of robot");
        Console.WriteLine("  <float> x 9: move to postition (cartesian with fingers)");
        continue;
      }
      if (String.Compare(line, "stop") == 0) {
        Stop();
        continue;
      }
      if (String.Compare(line, "read") == 0) {
        GetData();
        for (int i=0; i<15; i++) Console.Write(_data[i]+"  ");
        Console.WriteLine();
        continue;
      }
      if (String.Compare(line, "read") == 0) {
        _jaco.ControlManager.StartControlAPI();
        continue;
      }
      string[] pos = line.Split(' ');
      if (pos.Length < 9) continue;
      SetPosition(true, float.Parse(pos[0]), float.Parse(pos[1]), float.Parse(pos[2]), float.Parse(pos[3]), float.Parse(pos[4]), float.Parse(pos[5]), float.Parse(pos[6]), float.Parse(pos[7]), float.Parse(pos[8]));
    }
    return true;
  }

  static bool SaveZonesToProfile (string profile)
  {
    _jaco.ConfigurationsManager.SerializeZoneConfiguration(profile);
    Console.WriteLine(_info + "Saved zone configuration in profile \"" + profile + "\".");
    return true;
  }

  static bool LoadZonesFromProfile (string profile)
  {
    try {
      _jaco.ConfigurationsManager.DeserializeZoneConfiguration(profile);
      Console.WriteLine(_info + "Loaded zone configuration from profile \"" + profile + "\".");
      Console.WriteLine(_info + "You will need to reboot the robot for changes to take effect.");
      _zones = _jaco.ConfigurationsManager.GetProtectionZones();
      PrintZones();
      return true;
    } catch (Exception) {
      Console.WriteLine(_error + "Could not load zone configuration from profile \"" + profile + "\".");
    }
    return false;
  }

  static bool SaveZonesToFile (string filename)
  {
    int num = _zones.NbZoneActive;
    StreamWriter file = new StreamWriter(filename);
    for (int z=0; z<num; z++) {
      file.WriteLine(_info + "# Zone " + z);
      for (int p=0; p<4; p++) {
        CVectorEuler point = _zones.Zones[z].ZoneShape.Point[p];
        file.WriteLine(point.Position[0] + " " + point.Position[1] + " " + point.Position[2]);
      }
      file.WriteLine(_zones.Zones[z].ZoneLimitation.AngularSpeed + " " + _zones.Zones[z].ZoneLimitation.LinearSpeed + " " + _zones.Zones[z].ZoneShape.Point[4].Position[2]); 
    }
    file.Close();
    Console.WriteLine(_info + "Wrote zone configuration to file \"" + filename + "\".");
    return true;
  }

  static bool LoadZonesFromFile (string filename)
  {
    try {
      string[] file = File.ReadAllLines(filename);
      int z;
      for (z=0; z*6+5<file.Length; z++) {
        CPoint3d[] points = new CPoint3d[5];
        for (int p=0; p<5; p++) {
          points[p] = new CPoint3d();
          string[] pos = file[z*6+p+1].Split(' ');
          points[p].X = float.Parse(pos[0]);
          points[p].Y = float.Parse(pos[1]);
          points[p].Z = float.Parse(pos[2]);
        }
        _zones.Zones[z] = CZone.CreateBaseZoneXY(points[0], points[1], points[2], points[3], points[4].Z, points[4].X, points[4].Y);
      }
      _zones.NbZoneActive = z;
      _jaco.ConfigurationsManager.SetProtectionZones(_zones);
      Console.WriteLine(_info + "Read zone configuration from file \"" + filename + "\".");
      Console.WriteLine(_info + "You will need to reboot the robot for changes to take effect.");
      PrintZones();
      return true;
    } catch (Exception) {
      Console.WriteLine(_error + "Could not read zone configuration from file \"" + filename + "\".");
    }
    return false;
  }

  static void PrintZones ()
  {
    int znum = _zones.NbZoneActive;
    Console.WriteLine(_info + "Active Zones: " + znum);
    for (int z=0; z<znum; z++) {
      Console.WriteLine(_info + "Zone " + z + " (" + _zones.Zones[z].ZoneShape.ShapeType + ")");
      for (int p=0; p<4; p++) {
        CVectorEuler point = _zones.Zones[z].ZoneShape.Point[p];
        Console.WriteLine(_info + "    Base " + p + ":  " + point.Position[0] + "  " + point.Position[1] + "  " + point.Position[2]);
      }
      Console.WriteLine(_info + "    Height:  " + _zones.Zones[z].ZoneShape.Point[4].Position[2]);
      Console.WriteLine(_info + "    Angular Limit:  " + _zones.Zones[z].ZoneLimitation.AngularSpeed);
      Console.WriteLine(_info + "    Linear Limit:   " + _zones.Zones[z].ZoneLimitation.LinearSpeed);
    }
  }

}

}

