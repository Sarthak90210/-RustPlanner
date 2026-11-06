using System;
using System.Drawing;
using System.Linq;
using System.Collections.Generic;
using System.Windows.Forms;

public enum MAV_SENSOR_ORIENTATION { MAV_SENSOR_ROTATION_NONE=0, MAV_SENSOR_ROTATION_YAW_45=1, MAV_SENSOR_ROTATION_YAW_90=2, MAV_SENSOR_ROTATION_YAW_135=3, MAV_SENSOR_ROTATION_YAW_180=4, MAV_SENSOR_ROTATION_YAW_225=5, MAV_SENSOR_ROTATION_YAW_270=6, MAV_SENSOR_ROTATION_YAW_315=7, MAV_SENSOR_ROTATION_CUSTOM=100 }
public enum Rotation { ROTATION_NONE=0, ROTATION_YAW_45=1, ROTATION_YAW_90=2, ROTATION_YAW_135=3, ROTATION_YAW_180=4, ROTATION_YAW_225=5, ROTATION_YAW_270=6, ROTATION_YAW_315=7 }
public enum Firmwares { ArduCopter2, ArduPlane }
public class ParentState { public State cs = new State(); }
public class State { public Firmwares firmware; }
public class Datum { public MAV_SENSOR_ORIENTATION Orientation; public double Distance, Angle, Size; }
public class Directions { public List<Datum> Data = new List<Datum>(); public List<Datum> GetRaw() { return Data; } }
public static class Resources { public static Bitmap quadicon; }
// Same planar rotation equations as Utilities/Vector3.cs rotate(double), with
// explicit yaw enum conversion. Non-planar rotations are absent from this fixture.
public class Vector3 {
    public double x,y,z; public double X { get {return x;} } public double Y { get {return y;} }
    public Vector3(double a,double b,double c) { x=a;y=b;z=c; }
    public double length() { return Math.Sqrt(x*x+y*y+z*z); }
    public void rotate(double degrees) { double a=degrees*Math.PI/180; double next=x*Math.Cos(a)-y*Math.Sin(a); y=x*Math.Sin(a)+y*Math.Cos(a); x=next; }
    public void rotate(Rotation r) { rotate((double)r*45); }
}
public class ProximityPaintHarness {
    public int Width,Height;
    public Color BackColor=Color.FromArgb(0x26,0x27,0x28);
    public string Text;
    public float screenradius=500,mavsize=80;
    ParentState _parent=new ParentState(); Directions _dS=new Directions();
    public bool Copter { set { _parent.cs.firmware=value?Firmwares.ArduCopter2:Firmwares.ArduPlane; } }
    public void Add(int orientation,double angle,double size,double distance) { _dS.Data.Add(new Datum { Orientation=(MAV_SENSOR_ORIENTATION)orientation, Angle=angle, Size=size, Distance=distance }); }
    public void Render(string path,string icon) {
        using (Resources.quadicon=new Bitmap(icon))
        using (var bitmap=new Bitmap(Width,Height))
        using (var graphics=Graphics.FromImage(bitmap)) {
            bitmap.SetResolution(96,96);
            Temp_Paint(this,new PaintEventArgs(graphics,new Rectangle(0,0,Width,Height)));
            bitmap.Save(path,System.Drawing.Imaging.ImageFormat.Png);
        }
    }
    /*REFERENCE_PAINT_METHOD*/
}
