/*
 * Copyright (c) 1999 - 2001, Artur Merke <amerke@ira.uka.de> 
 *
 * This file is part of FrameView2d.
 *
 * FrameView2d is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * FrameView2d is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with FrameView2d; see the file COPYING.  If not, write to
 * the Free Software Foundation, 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#ifndef _CODER_H_
#define _CODER_H_

#include "rgbcolor.h"
#include "vector2d.h"
#include "object2d.h"
#include "multi.h"
#include "rwbin.h"
#include "builder_base.h"

inline void wbin(ostream& o, const RGBcolor& col) {
  wbin(o, col.red);
  wbin(o, col.green);
  wbin(o, col.blue);
}

inline void rbin(istream& i,RGBcolor& col) {
  rbin(i, col.red);
  rbin(i, col.green);
  rbin(i, col.blue);
}

inline void wbin(ostream& o, const Angle& ang) {
  wbin(o, ang.get_value());
}

inline void rbin(istream& i, Angle& ang) {
  double d;
  rbin(i, d);
  ang= Angle(d);
}

inline void wbin(ostream& o, const Point2d& p) {
  wbin(o, p.x); 
  wbin(o, p.y);
}

inline void rbin(istream& i,Point2d& p) {
  rbin(i, p.x); 
  rbin(i, p.y);
}

inline void wbin(ostream& o, const Line2d& line) {
  wbin(o, line.p1); 
  wbin(o, line.p2);
}

inline void rbin(istream& i,Line2d& line) {
  rbin(i, line.p1); 
  rbin(i, line.p2);
}

inline void wbin(ostream& o, const Circle2d& circ) {
  wbin(o, circ.center); 
  wbin(o, circ.radius);
}

inline void rbin(istream& i, Circle2d& circ) {
  rbin(i, circ.center); 
  rbin(i, circ.radius);
}

inline void wbin(ostream& o, const CircleArc2d& circ) {
  wbin(o, circ.center); 
  wbin(o, circ.radius); 
  wbin(o, circ.ang1); 
  wbin(o, circ.ang2);
}

inline void rbin(istream& i, CircleArc2d& circ) {
  rbin(i, circ.center);
  rbin(i, circ.radius); 
  rbin(i, circ.ang1);
  rbin(i, circ.ang2);
}

template<class T> 
inline void wbin(ostream& o, const Multi<T>& mult) {
  wbin(o, mult.cur_size);
  for (int k=0; k< mult.cur_size; k++)
    wbin(o, mult.tab[k]);
}

template<class T>
inline void rbin(istream& i, Multi<T>& mult) {
  int dum;
  rbin(i, dum);
  if (i.fail()) return;

  mult.cur_size= dum;
  if (mult.cur_size>mult.max_size) {
    if (mult.tab) delete[] mult.tab;
    mult.max_size= mult.cur_size;
    mult.tab= new T[mult.max_size];
  }
  for (int k=0; k<mult.cur_size; k++)
    rbin(i, mult.tab[k]);
}

class EnDeCoderBin {
 public:
  bool build_from_binary(BuilderBase * build, istream & in);

  static const int cmd_insert_frame       = 1;

  static const int cmd_insert_point       = 2; 
  static const int cmd_insert_points      = 3; 

  static const int cmd_insert_line        = 4;   
  static const int cmd_insert_lines       = 5;   

  static const int cmd_insert_circle      = 6;
  static const int cmd_insert_circles     = 7;
  static const int cmd_insert_f_circle    = 8; //f means filled
  static const int cmd_insert_f_circles   = 9; //f means filled

  static const int cmd_insert_circlearc   = 10;
  static const int cmd_insert_circlearcs  = 11;
  static const int cmd_insert_f_circlearc = 12;
  static const int cmd_insert_f_circlearcs= 13;

  static const int cmd_insert_polyline    = 14;

  static const int cmd_insert_polygon     = 15;
  static const int cmd_insert_f_polygon   = 16;

  static const int cmd_insert_string      = 17;


  static const int cmd_set_object_visible = 18;
  static const int cmd_set_object_layer   = 19;
  static const int cmd_set_object_color   = 20;

  static const int cmd_set_frame_visible  = 21;
  static const int cmd_set_frame_layer    = 22;
  static const int cmd_set_frame_pos      = 23;
  static const int cmd_set_frame_ang      = 24;
  static const int cmd_set_frame_pos_ang  = 25;

  static const int cmd_remove_frame       = 26;
  static const int cmd_remove_object      = 27;

  static const int cmd_empty_frame        = 28;

/*   static const int cmd_rotate_frame       = 10; */
/*   static const int cmd_translate_frame    = 11; */

  bool get_cmd_type(istream &i,int &cmd) {
    rbin(i,cmd);
    return !i.eof();
  }
  ////
  bool set_cmd_insert_frame(ostream &o, int parent_frame,int this_frame,
			    const Point2d& pos , const Angle& ang, int layer) {
    wbin(o,cmd_insert_frame); wbin(o,parent_frame); wbin(o,this_frame); wbin(o,pos); wbin(o, ang); wbin(o,layer);
    return !o.fail();
  }

  bool get_cmd_insert_frame(istream &i,int & parent_frame,int & this_frame,
			    Point2d& pos, Angle& angle,int & layer) {
    rbin(i,parent_frame); rbin(i,this_frame); rbin(i,pos); rbin(i, angle); rbin(i,layer);
    return !i.fail();
  }
  ////
  bool set_cmd_insert_point(ostream &o, int parent_frame, int this_object,
			    const Point2d& obj, int layer, const RGBcolor &col) {
    wbin(o,cmd_insert_point); wbin(o,parent_frame); wbin(o,this_object); wbin(o,obj); wbin(o,layer); wbin(o,col);
    return !o.fail();
  }

  bool get_cmd_insert_point(istream &i,int & parent_frame,int & this_object,
			    Point2d& obj,int & layer, RGBcolor & col) {
    rbin(i,parent_frame); rbin(i,this_object); rbin(i,obj); rbin(i,layer); rbin(i,col);
    return !i.fail();
  }

  bool set_cmd_insert_points(ostream &o,int parent_frame,int this_object,
			     const Multi<Point2d>& mul,int layer, const RGBcolor &col) {
    wbin(o,cmd_insert_points); wbin(o,parent_frame); wbin(o,this_object); wbin(o,mul); wbin(o,layer); wbin(o,col);
    return !o.fail();
  }

  bool get_cmd_insert_points(istream &i,int & parent_frame,int & this_object,
			     Multi<Point2d>& mul,int & layer, RGBcolor & col) {
    rbin(i,parent_frame); rbin(i,this_object); rbin(i,mul); rbin(i,layer); rbin(i,col);
    return !i.fail();
  }
  ////
  bool set_cmd_insert_line(ostream &o, int parent_frame, int this_object,
			    const Line2d& obj, int layer, const RGBcolor &col) {
    wbin(o,cmd_insert_line); wbin(o,parent_frame); wbin(o,this_object); wbin(o,obj); wbin(o,layer); wbin(o,col);
    return !o.fail();
  }

  bool get_cmd_insert_line(istream &i,int & parent_frame,int & this_object,
			    Line2d& obj,int & layer, RGBcolor & col) {
    rbin(i,parent_frame); rbin(i,this_object); rbin(i,obj); rbin(i,layer); rbin(i,col);
    return !i.fail();
  }

  bool set_cmd_insert_lines(ostream &o,int parent_frame,int this_object,
			     const Multi<Line2d>& mul,int layer, const RGBcolor &col) {
    wbin(o,cmd_insert_lines); wbin(o,parent_frame); wbin(o,this_object); wbin(o,mul); wbin(o,layer); wbin(o,col);
    return !o.fail();
  }

  bool get_cmd_insert_lines(istream &i,int & parent_frame,int & this_object,
			     Multi<Line2d>& mul,int & layer, RGBcolor & col) {
    rbin(i,parent_frame); rbin(i,this_object); rbin(i,mul); rbin(i,layer); rbin(i,col);
    return !i.fail();
  }
  ////
  bool set_cmd_insert_circle(ostream &o, int parent_frame, int this_object,
			    const Circle2d& obj, int layer, const RGBcolor &col) {
    wbin(o,cmd_insert_circle); wbin(o,parent_frame); wbin(o,this_object); wbin(o,obj); wbin(o,layer); wbin(o,col);
    return !o.fail();
  }

  bool get_cmd_insert_circle(istream &i,int & parent_frame,int & this_object,
			    Circle2d& obj,int & layer, RGBcolor & col) {
    rbin(i,parent_frame); rbin(i,this_object); rbin(i,obj); rbin(i,layer); rbin(i,col);
    return !i.fail();
  }

  bool set_cmd_insert_circles(ostream &o,int parent_frame,int this_object,
			     const Multi<Circle2d>& mul,int layer, const RGBcolor &col) {
    wbin(o,cmd_insert_circles); wbin(o,parent_frame); wbin(o,this_object); wbin(o,mul); wbin(o,layer); wbin(o,col);
    return !o.fail();
  }

  bool get_cmd_insert_circles(istream &i,int & parent_frame,int & this_object,
			     Multi<Circle2d>& mul,int & layer, RGBcolor & col) {
    rbin(i,parent_frame); rbin(i,this_object); rbin(i,mul); rbin(i,layer); rbin(i,col);
    return !i.fail();
  }
  
  bool set_cmd_insert_f_circle(ostream &o, int parent_frame, int this_object,
			    const Circle2d& obj, int layer, const RGBcolor &col) {
    wbin(o,cmd_insert_f_circle); wbin(o,parent_frame); wbin(o,this_object); wbin(o,obj); wbin(o,layer); wbin(o,col);
    return !o.fail();
  }

  bool get_cmd_insert_f_circle(istream &i,int & parent_frame,int & this_object,
			    Circle2d& obj,int & layer, RGBcolor & col) {
    rbin(i,parent_frame); rbin(i,this_object); rbin(i,obj); rbin(i,layer); rbin(i,col);
    return !i.fail();
  }

  bool set_cmd_insert_f_circles(ostream &o,int parent_frame,int this_object,
			     const Multi<Circle2d>& mul,int layer, const RGBcolor &col) {
    wbin(o,cmd_insert_f_circles); wbin(o,parent_frame); wbin(o,this_object); wbin(o,mul); wbin(o,layer); wbin(o,col);
    return !o.fail();
  }

  bool get_cmd_insert_f_circles(istream &i,int & parent_frame,int & this_object,
			     Multi<Circle2d>& mul,int & layer, RGBcolor & col) {
    rbin(i,parent_frame); rbin(i,this_object); rbin(i,mul); rbin(i,layer); rbin(i,col);
    return !i.fail();
  }
  ////
  bool set_cmd_insert_circlearc(ostream &o, int parent_frame, int this_object,
			    const CircleArc2d& obj, int layer, const RGBcolor &col) {
    wbin(o,cmd_insert_circlearc); wbin(o,parent_frame); wbin(o,this_object); wbin(o,obj); wbin(o,layer); wbin(o,col);
    return !o.fail();
  }

  bool get_cmd_insert_circlearc(istream &i,int & parent_frame,int & this_object,
			    CircleArc2d& obj,int & layer, RGBcolor & col) {
    rbin(i,parent_frame); rbin(i,this_object); rbin(i,obj); rbin(i,layer); rbin(i,col);
    return !i.fail();
  }

  bool set_cmd_insert_circlearcs(ostream &o,int parent_frame,int this_object,
			     const Multi<CircleArc2d>& mul,int layer, const RGBcolor &col) {
    wbin(o,cmd_insert_circlearcs); wbin(o,parent_frame); wbin(o,this_object); wbin(o,mul); wbin(o,layer); wbin(o,col);
    return !o.fail();
  }

  bool get_cmd_insert_circlearcs(istream &i,int & parent_frame,int & this_object,
			     Multi<CircleArc2d>& mul,int & layer, RGBcolor & col) {
    rbin(i,parent_frame); rbin(i,this_object); rbin(i,mul); rbin(i,layer); rbin(i,col);
    return !i.fail();
  }
  
  bool set_cmd_insert_f_circlearc(ostream &o, int parent_frame, int this_object,
			    const CircleArc2d& obj, int layer, const RGBcolor &col) {
    wbin(o,cmd_insert_f_circlearc); wbin(o,parent_frame); wbin(o,this_object); wbin(o,obj); wbin(o,layer); wbin(o,col);
    return !o.fail();
  }

  bool get_cmd_insert_f_circlearc(istream &i,int & parent_frame,int & this_object,
			    CircleArc2d& obj,int & layer, RGBcolor & col) {
    rbin(i,parent_frame); rbin(i,this_object); rbin(i,obj); rbin(i,layer); rbin(i,col);
    return !i.fail();
  }

  bool set_cmd_insert_f_circlearcs(ostream &o,int parent_frame,int this_object,
			     const Multi<CircleArc2d>& mul,int layer, const RGBcolor &col) {
    wbin(o,cmd_insert_f_circlearcs); wbin(o,parent_frame); wbin(o,this_object); wbin(o,mul); wbin(o,layer); wbin(o,col);
    return !o.fail();
  }

  bool get_cmd_insert_f_circlearcs(istream &i,int & parent_frame,int & this_object,
			     Multi<CircleArc2d>& mul,int & layer, RGBcolor & col) {
    rbin(i,parent_frame); rbin(i,this_object); rbin(i,mul); rbin(i,layer); rbin(i,col);
    return !i.fail();
  }
  ////
  bool set_cmd_insert_polyline(ostream &o,int parent_frame,int this_object,
			     const Multi<Point2d>& mul,int layer, const RGBcolor &col) {
    wbin(o,cmd_insert_polyline); wbin(o,parent_frame); wbin(o,this_object); wbin(o,mul); wbin(o,layer); wbin(o,col);
    return !o.fail();
  }

  bool get_cmd_insert_polyline(istream &i,int & parent_frame,int & this_object,
			     Multi<Point2d>& mul,int & layer, RGBcolor & col) {
    rbin(i,parent_frame); rbin(i,this_object); rbin(i,mul); rbin(i,layer); rbin(i,col);
    return !i.fail();
  }
  ////
  bool set_cmd_insert_polygon(ostream &o,int parent_frame,int this_object,
			     const Multi<Point2d>& mul,int layer, const RGBcolor &col) {
    wbin(o,cmd_insert_polygon); wbin(o,parent_frame); wbin(o,this_object); wbin(o,mul); wbin(o,layer); wbin(o,col);
    return !o.fail();
  }

  bool get_cmd_insert_polygon(istream &i,int & parent_frame,int & this_object,
			     Multi<Point2d>& mul,int & layer, RGBcolor & col) {
    rbin(i,parent_frame); rbin(i,this_object); rbin(i,mul); rbin(i,layer); rbin(i,col);
    return !i.fail();
  }

  bool set_cmd_insert_f_polygon(ostream &o,int parent_frame,int this_object,
			     const Multi<Point2d>& mul,int layer, const RGBcolor &col) {
    wbin(o,cmd_insert_f_polygon); wbin(o,parent_frame); wbin(o,this_object); wbin(o,mul); wbin(o,layer); wbin(o,col);
    return !o.fail();
  }

  bool get_cmd_insert_f_polygon(istream &i,int & parent_frame,int & this_object,
			     Multi<Point2d>& mul,int & layer, RGBcolor & col) {
    rbin(i,parent_frame); rbin(i,this_object); rbin(i,mul); rbin(i,layer); rbin(i,col);
    return !i.fail();
  }
  ////
  bool set_cmd_insert_string(ostream &o,int parent_frame,int this_object,const Point2d& pos,
			     const Multi<char>& mul,int layer, const RGBcolor &col) {
    wbin(o,cmd_insert_string); wbin(o,parent_frame); wbin(o,this_object); wbin(o,pos); wbin(o,mul); wbin(o,layer); wbin(o,col);
    return !o.fail();
  }

  bool get_cmd_insert_string(istream &i,int & parent_frame,int & this_object, Point2d& pos,
			     Multi<char>& mul,int & layer, RGBcolor & col) {
    rbin(i,parent_frame); rbin(i,this_object); rbin(i,pos); rbin(i,mul); rbin(i,layer); rbin(i,col);
    return !i.fail();
  }
  ////
  bool set_cmd_set_object_visible(ostream &o,int frame, int this_object,int vis) {
    wbin(o,cmd_set_object_visible); wbin(o,frame); wbin(o,this_object); wbin(o,vis);
    return !o.fail();
  }

  bool get_cmd_set_object_visible(istream &i,int & frame,int & this_object,int & vis) {
    rbin(i,frame); rbin(i,this_object); rbin(i,vis);
    return !i.fail();
  }
  ////
  bool set_cmd_set_object_layer(ostream &o,int frame, int this_object,int layer) {
    wbin(o,cmd_set_object_layer); wbin(o,frame); wbin(o,this_object); wbin(o,layer);
    return !o.fail();
  }

  bool get_cmd_set_object_layer(istream &i,int & frame,int & this_object,int & layer) {
    rbin(i,frame); rbin(i,this_object); rbin(i,layer);
    return !i.fail();
  }
  ////
  bool set_cmd_set_object_color(ostream &o,int frame, int this_object,const RGBcolor& col) {
   wbin(o,cmd_set_object_color); wbin(o,frame); wbin(o,this_object); wbin(o,col);
    return !o.fail();
  }

  bool get_cmd_set_object_color(istream &i,int & frame,int & this_object,RGBcolor & col) {
    rbin(i,frame); rbin(i,this_object); rbin(i,col);
    return !i.fail();
  }
  ////
  bool set_cmd_set_frame_visible(ostream &o, int this_frame,int visible) {
    wbin(o,cmd_set_frame_visible); wbin(o,this_frame); wbin(o,visible);
    return !o.fail();
  }

  bool get_cmd_set_frame_visible(istream &i,int & this_frame,int & visible) {
    rbin(i,this_frame); rbin(i,visible);
    return !i.fail();
  }
  ////
  bool set_cmd_set_frame_layer(ostream &o, int this_frame,int layer) {
    wbin(o,cmd_set_frame_layer); wbin(o,this_frame); wbin(o,layer);
    return !o.fail();
  }

  bool get_cmd_set_frame_layer(istream &i,int & this_frame,int & layer) {
    rbin(i,this_frame); rbin(i,layer);
    return !i.fail();
  }
  ////
  bool set_cmd_set_frame_pos(ostream &o, int this_frame, const Point2d& pos) {
    wbin(o,cmd_set_frame_pos); wbin(o,this_frame); wbin(o,pos);
    return !o.fail();
  }

  bool get_cmd_set_frame_pos(istream &i,int & this_frame,Point2d& pos) {
    rbin(i,this_frame); rbin(i,pos);
    return !i.fail();
  }
  ////
  bool set_cmd_set_frame_ang(ostream &o, int this_frame, const Angle& ang) {
    wbin(o,cmd_set_frame_ang); wbin(o,this_frame); wbin(o,ang);
    return !o.fail();
  }

  bool get_cmd_set_frame_ang(istream &i,int & this_frame,Angle& ang) {
    rbin(i,this_frame); rbin(i,ang);
    return !i.fail();
  }
  ////
  bool set_cmd_set_frame_pos_ang(ostream &o, int this_frame, const Point2d& pos, const Angle& ang) {
    wbin(o,cmd_set_frame_pos_ang); wbin(o,this_frame); wbin(o,pos); wbin(o,ang);
    return !o.fail();
  }

  bool get_cmd_set_frame_pos_ang(istream &i,int & this_frame,Point2d& pos, Angle& ang) {
    rbin(i,this_frame); rbin(i,pos); rbin(i,ang);
    return !i.fail();
  }
  ////
  bool set_cmd_remove_frame(ostream &o, int this_frame) {
    wbin(o,cmd_remove_frame); wbin(o,this_frame);
    return !o.fail();
  }

  bool get_cmd_remove_frame(istream &i,int & this_frame) {
    rbin(i,this_frame);
    return !i.fail();
  }
  ////
  bool set_cmd_remove_object(ostream &o, int frame, int this_object) {
    wbin(o,cmd_remove_object); wbin(o,frame); wbin(o,this_object);
    return !o.fail();
  }

  bool get_cmd_remove_object(istream &i,int & frame,int & this_object) {
    rbin(i,frame); rbin(i,this_object);
    return !i.fail();
  }
  ////
  bool set_cmd_empty_frame(ostream &o, int this_frame) {
    wbin(o,cmd_empty_frame); wbin(o,this_frame);
    return !o.fail();
  }

  bool get_cmd_empty_frame(istream &i,int & this_frame) {
    rbin(i,this_frame);
    return !i.fail();
  }
  ////
};

#endif
