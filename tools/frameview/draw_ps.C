#include <iostream>

#include "rgbcolor.h"
#include "object2d.h"

void ADrawPoint(double a1,double a2) {
  cout << "\n% ADrawPoint"
       << "\n% not yet implemented";
  //ADrawCircle(a1,a2,0.1);
}

void ADrawPoints(int num, const Point2d* const vec) {
  cout << "\n% ADrawPoints"
       << "\n% not yet implemented";
}
void ADrawLine(double a1, double a2, double b1, double b2) {
  cout << "\n% ADrawLine"
       << "\n" << a1 << " " << a2 << " moveto"
       << "\n" << b1 << " " << b2 << " lineto stroke";
}

void ADrawLines(int num, const Line2d* const vec) {
  cout << "\n% ADrawLines";
  for (int i=0; i<num; i++) 
    cout << "\n" << vec[i].p1.x << " " << vec[i].p1.y << " moveto" 
	 << " " <<  vec[i].p2.x << " " << vec[i].p2.y << " lineto";
  cout << "\nstroke";
}

void ADrawCircle(double a1, double a2, double r) {
  cout << "\n% ADrawCircle"
       << "\n" << a1 << " " << a2 << " " << r << " 0 360 arc stroke";
}

void ADrawCircles(int num, const Circle2d* const vec) {
  cout << "\n% ADrawCircles";
  for (int i=0; i<num; i++) 
    cout << "\n" << vec[i].center.x << " " << vec[i].center.y << " " << vec[i].radius << " 0 360 arc";
  cout << "\nstroke";
}

void AFillCircle(double a1, double a2, double r) {
  cout << "\n% AFillCircle"
       << "\n" << a1 << " " << a2 << " " << r << " 120 360 arc closepath"
       << "\nfill stroke";
}

void AFillCircles(int num, const Circle2d* const vec) {
  cout << "\n% AFillCircles";
  for (int i=0; i<num; i++) 
    cout << "\n" << vec[i].center.x << " " << vec[i].center.y << " " << vec[i].radius << " 0 360 arc";
  cout << "\nfill stroke";
}

void ADrawCircleArc(double a1,double a2,double r,const Angle& ang1, const Angle& ang2) {
  float angle1, angle2;
  angle1= ang1.get_value() * 180.0 / M_PI;
  angle2= ang2.get_value() * 180.0 / M_PI;
  cout << "\n% ADrawCircleArc"
       << "\n" << a1 << " " << a2 << " " << r << " " << angle1 << " " << angle2 << " arc stroke";
}

void ADrawCircleArcs(int num, const CircleArc2d* const vec) {
  float angle1, angle2;
  cout << "\n% ADrawCircleArcs";
  for (int i=0; i<num; i++) {
    angle1= vec[i].ang1.get_value() * 180.0 / M_PI;
    angle2= vec[i].ang2.get_value() * 180.0 / M_PI;
    cout << "\n" << vec[i].center.x << " " << vec[i].center.y << " " << vec[i].radius 
	 << " " << angle1 << " " << angle2 << " arc";

  }
  cout << "\nstroke";
}

void AFillCircleArc(double a1,double a2,double r,const Angle& ang1, const Angle& ang2) {
  float angle1, angle2;
  angle1= ang1.get_value() * 180.0 / M_PI;
  angle2= ang2.get_value() * 180.0 / M_PI;
  cout << "\n% AFillCircleArc"
       << "\n" << a1 << " " << a2 << " " << r << " " << angle1 << " " << angle2 << " arc closepath fill stroke";
}

void AFillCircleArcs(int num, const CircleArc2d* const vec) {
  float angle1, angle2;
  cout << "\n% AFillCircleArcs";
  for (int i=0; i<num; i++) {
    angle1= vec[i].ang1.get_value() * 180.0 / M_PI;
    angle2= vec[i].ang2.get_value() * 180.0 / M_PI;
    cout << "\n" << vec[i].center.x << " " << vec[i].center.y << " " << vec[i].radius 
	 << " " << angle1 << " " << angle2 << " arc closepath fill";

  }
  cout << "\nstroke";
}

void ADrawPolyline(int num, const Point2d * const vec) {
  cout << "\n% ADrawPolyline";
  if (num <= 0)
    return;
  cout << "\n" << vec[0].x << " " << vec[0].y << " moveto";
  for (int i=1; i<num; i++) 
    cout  << "\n" <<  vec[i].x << " " << vec[i].y << " lineto";
  cout << "\nstroke";
}

void ADrawPolygon(int num, const Point2d * const vec) {
  cout << "\n% ADrawPolygon";
  if (num <= 0)
    return;
  cout << "\n" << vec[0].x << " " << vec[0].y << " moveto";
  for (int i=1; i<num; i++) 
    cout  << "\n" <<  vec[i].x << " " << vec[i].y << " lineto";
  cout << "\nclosepath stroke";
}

void AFillPolygon(int num, const Point2d * const vec) {
  cout << "\n% ADrawPolygon";
  if (num <= 0)
    return;
  cout << "\n" << vec[0].x << " " << vec[0].y << " moveto";
  for (int i=1; i<num; i++) 
    cout  << "\n" <<  vec[i].x << " " << vec[i].y << " lineto";
  cout << "\nclosepath fill stroke";
}

void ASetForground(const RGBcolor & rgb) {
  float r,g,b;
  if (rgb.red == 255) 
    r=1.0;
  else if (rgb.red)
    r= 255.0/float(rgb.red);
  else 
    r= 0.0;
  
  if (rgb.green == 255) 
    g=1.0;
  else if (rgb.green)
    g= 255.0/float(rgb.green);
  else 
    g= 0.0;
  
  if (rgb.blue == 255) 
    b=1.0;
  else if (rgb.blue)
    b= 255.0/float(rgb.blue);
  else 
    b= 0.0;

  cout << "\n% new color"
       << "\n" << r << " " << g << " " << b << " setrgbcolor";
}

void page_begin(ostream & o) {
  o <<  "%!PS\n\n"
  << "% PostScript normally uses units of \"points\" for placing graphics on the page.\n"
  << "% Here we change it to work with centimeters."
  << "matrix currentmatrix /originmat exch def [28.3465 0 0 28.3465 10.5 100.0] originmat matrix concatmatrix setmatrix\n"
  << "0.01 setlinewidth";
}

void page_end(ostream & o) {
  o <<  "\n showpage\n";
}

int main() {
  page_begin(cout);
  ADrawLine(0,0,10,7);
  ADrawCircle(10,7,3);
  ASetForground(RGBcolor(255,0,0));
  AFillCircle(10,15,2);
  ASetForground(RGBcolor(0,0,0));
  ADrawCircleArc(14,15,1,0.0, 3.14);
  AFillCircleArc(14,15,1,3.14, 6.28);

  Line2d lines[2];
  lines[0]= Line2d(Point2d(1,0),Point2d(1,5));
  lines[1]= Line2d(Point2d(2,0),Point2d(2,5));
  ADrawLines(2,lines);

  CircleArc2d circles[2];
  circles[0]= CircleArc2d(Point2d(6,0),1,0,3.14);
  circles[1]= CircleArc2d(Point2d(9,0),1.0,3.14,1.5);
  AFillCircleArcs(2,circles);

  Point2d points[3]= { Point2d(0,5), Point2d(3,10), Point2d(1,14) };
  //ADrawPolyline(3,points);
  AFillPolygon(3,points);

  page_end(cout);
  return 0;
}


