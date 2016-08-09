
/*
n++: neural network simulator (multi-layer-perceptrons)

Copyright (c) 1994, 2004, Martin Riedmiller, University of Osnabrueck
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

/******************************************************************************/

/* N++ : C++ class neural network simulator                                   */
/* (c) 1994 Martin Riedmiller                                                 */
/* last changed: 22.06.95                                                      */

/* File: n++.cc                                                                */
/* Purpose: Implementation of n++ class neural network                         */

/******************************************************************************/

#ifdef MSDOS
#include "npp.h"
#else
#include "n++.h"
#endif
#include "functions.h"  /* activation, update, propagate,... */

/********************************************************************************/

/*       network creation functions                                             */

/********************************************************************************/

Net::Net( void ){
// set defaults
  double default_params[MAX_PARAMS];
  int i;

  topo_data.unit_count = topo_data.in_count = 
    topo_data.out_count = topo_data.hidden_count = 0;
  topo_data.layer_count = 0;
  forward_pass_f = prop_forward;
  backward_pass_f = prop_backward;
  /* set default weight update function */
  for(i=0;i<MAX_PARAMS;i++)
    default_params[i] = 0.0;
  default_params[0] = 0.1; 
  set_update_f(BP,default_params); 
  scale_list_in = scale_list_out = NULL;
  unit=NULL;
  scaled_in_vec = in_vec = out_vec = NULL;
  network_type = FEEDFORWARD;
}

Net::~Net() 
{
  delete_structure();
}

void Net::delete_structure()
{
  if (in_vec!=0)
    delete[] in_vec;
  if (out_vec!=0)
    delete[] out_vec;
  if (scaled_in_vec!=0)
    delete[] scaled_in_vec;

  if (unit != 0){
    weight_typ *wp1, *wp2;
    for (int i=0; i <= topo_data.unit_count; i++) {
      wp1 = unit[i].weights;
      for(;wp1!=0;  wp1 = wp2){
	wp2 = wp1->next;
	delete wp1;
      }
    }
    delete[] unit;
  }

  scale_typ *scp1, *scp2;
  scp1 = scale_list_in;
  for(;scp1!=0; scp1=scp2){
    scp2 = scp1->next;
    delete scp1;
  }
  scp1 = scale_list_out;
  for(;scp1!=0; scp1=scp2){
    scp2 = scp1->next;
    delete scp1;
  }
  
  /* ridi 11/96: reset */
  topo_data.unit_count = topo_data.in_count = 
    topo_data.out_count = topo_data.hidden_count = 0;
  topo_data.layer_count = 0;
  scale_list_in = scale_list_out = NULL;
  unit=NULL;
  scaled_in_vec = in_vec = out_vec = NULL;
}

int Net::create_layers(int layers, int layer_nodes[]){
  int i, start;

  if(topo_data.unit_count){
    fprintf(stderr,"Can't create layers - network already defined\n");
    return(NPP_ERROR);
  }
  /* set topolgy variables */
  topo_data.layer_count = layers;
  topo_data.in_count = layer_nodes[0];
  for(i=1,topo_data.hidden_count = 0;i<layers-1;i++)
    topo_data.hidden_count+=layer_nodes[i];   /* add number hidden units */
  topo_data.out_count = layer_nodes[layers-1];
  //  printf("n++ out_count set: %d",topo_data.out_count);
  /* allocate units */
  create_units(); /* create unit array */
  /* construct virtual segmentation in layers */
  for(i = 0, start = 1; i<topo_data.layer_count;i++){
    layer[i].unit = &unit[start];  /* points to the first element in layer */
    layer[i].unit_count = layer_nodes[i];
    start += layer_nodes[i];
  }
  in_vec = new FTYPE [topo_data.in_count];
  scaled_in_vec = new FTYPE [topo_data.in_count];
  out_vec = new FTYPE [topo_data.out_count];
  // compute deep encoder related data.
  topo_data.encoder_layer = topo_data.layer_count / 2;
  topo_data.encoder_count = layer_nodes[topo_data.encoder_layer];
  encoder_vec = new FTYPE [topo_data.encoder_count];

  printf("Assuming code layer is layer no. %d, with %d neurons, first encoder neuron: %d\n",
	 topo_data.encoder_layer,topo_data.encoder_count, 
	 layer[topo_data.encoder_layer].unit[0].unit_no);
   
  return(OK);
}

void Net::create_units()
{
  topo_data.unit_count = 
    topo_data.in_count+topo_data.hidden_count+topo_data.out_count; /* 'real' units */
  unit = new unit_typ[topo_data.unit_count+1]; /* allocate unit array real + bias */
  if(unit == NULL){
    fprintf(stderr,"Error: Can't allocate units\n");
    exit(0);
  }
  /* create bias unit */
  unit[0].unit_no = 0;    
  unit[0].out = (FTYPE) 1;   /* bias output is constant 1 */
  unit[0].weights = NULL;  /* no weights  */

  for(int i = 1; i<=topo_data.unit_count; i++){
    /* set defaults */
    unit[i].unit_no = i;    /* position in unit array */
    unit[i].dEdo = unit[i].net_in = unit[i].out = unit[i].new_out = unit[i].dEdnet = (FTYPE) 0;
    unit[i].weights = NULL;  /* no weights yet */
    for(int k=0;k<MAX_VARIABLES;k++)
      unit[i].variable[k]=(FTYPE) 0;
    if (IS_INPUT(i)){
      unit[i].deriv_f = linear_deriv;
      unit[i].act_id = LINEAR;
    }
    else{
      unit[i].act_f = logistic;
      unit[i].deriv_f = logistic_deriv;
      unit[i].act_id = LOGISTIC;
    }
  }
}

void Net::connect_units(int to_unit, int from_unit, FTYPE value){
  weight_typ *wptr;

  if(! IS_UNIT(to_unit) || (!IS_UNIT(from_unit)&& from_unit!=BIAS)  ){
    fprintf(stderr,"connect_units: not possible from %d to %d\n",from_unit, to_unit);
    return;
  }
  weight_typ *temp = new weight_typ;
  temp->from_unit = from_unit;
  temp->value = value;
  temp->dEdw = (FTYPE) 0;
  temp->delta = (FTYPE) 0;
  temp->next = NULL;
  for(int i=0;i<MAX_VARIABLES;i++)
    temp->variable[i]=(FTYPE) 0;
  if(unit[to_unit].weights==NULL){/* weight list empty */ 
    unit[to_unit].weights=temp;
  }
  else{ /* append weight at end of weight list */
    for(wptr=unit[to_unit].weights; wptr->next!=NULL;wptr=wptr->next){
      if(wptr->from_unit == from_unit){
	fprintf(stderr,"Connection %d->%d already exists\n",from_unit,to_unit);
	/* destroy(temp) */
	return;
      }
    } /* end of list reached */
    if(wptr->from_unit == from_unit){
      fprintf(stderr,"Connection %d->%d already exists\n",from_unit,to_unit);
      /* destroy(temp) */
      return;
    }
    else
      wptr->next = temp;
  }
}
  
void Net::connect_layers()
{
  int i,j,l;

  for(l=1;l<topo_data.layer_count;l++)             /* start with first hidden layer */
    for(i=0;i<layer[l].unit_count;i++){
      connect_units(layer[l].unit[i].unit_no,BIAS,0);
      for(j=0;j<layer[l-1].unit_count;j++)
	connect_units(layer[l].unit[i].unit_no,layer[l-1].unit[j].unit_no,0);
    }
}

void Net::connect_reservoir(double connectivity_rate)
{
  int i,j,l;

  printf("Connect Reservoir style. Requires one input, one outputlayer\n");

  for(l=1;l<topo_data.layer_count;l++)             /* start with first hidden layer */
    for(i=0;i<layer[l].unit_count;i++){
      connect_units(layer[l].unit[i].unit_no,BIAS,0);
      for(j=0;j<layer[l-1].unit_count;j++)  // connect to input
	connect_units(layer[l].unit[i].unit_no,layer[l-1].unit[j].unit_no,0);
      // reservoir connections:
      for(j=0;j<layer[l].unit_count;j++){  // connect layer completely to itself
	// self recurrent connection: change slowy
	/*
	if(i==j)
	  connect_units(layer[l].unit[i].unit_no,layer[l].unit[j].unit_no,0.0);
	*/
	if(drand48()>=1.0-connectivity_rate)
	  connect_units(layer[l].unit[i].unit_no,layer[l].unit[j].unit_no,0);
      }
    }
  
  network_type = RESERVOIR;
  forward_pass_f = prop_forward_reservoir;

  // print_net();
}

void Net::connect_shortcut()
{
  int i,j,l,k;

  for(l=1;l<topo_data.layer_count;l++)             /* start with first hidden layer */
    for(i=0;i<layer[l].unit_count;i++){
      connect_units(layer[l].unit[i].unit_no,BIAS,0);
      for(k=0;k<l;k++)
	for(j=0;j<layer[k].unit_count;j++)
	  connect_units(layer[l].unit[i].unit_no,layer[k].unit[j].unit_no,0);
    }
}

/********************************************************************************/

/*       input/output scaling                                                   */

/********************************************************************************/

void Net::insert_scale_element(scale_typ **scale_list,int position,
			    int scale_function, double param1,double param2,double param3)
/* create a new scale element and insert at beginning of input/or output scale-list */
{
  scale_typ *new_elem;  
  
  new_elem = new scale_typ;
  new_elem->scale_function = scale_function;
  new_elem->parameter1 = param1;
  new_elem->parameter2 = param2;
  new_elem->parameter3 = param3;
  new_elem->position = position;
  new_elem->next = *scale_list;  
  *scale_list = new_elem;
  //  printf("SCALE ELEMENT INSERTED\n");
}

void Net::apply_scaling(double* data_vector, scale_typ *scale_list)
{
  scale_typ *scale_elem;
  double new_value = 0;

  for(scale_elem = scale_list; scale_elem != NULL; scale_elem = scale_elem->next){
    switch(scale_elem->scale_function){
    case 0: /* symmetric scaling : if x>0: y=a*x if x<0:y=b*x */ 
      new_value=(data_vector[scale_elem->position-1]>0?
		 scale_elem->parameter1 * data_vector[scale_elem->position-1]:
		 scale_elem->parameter2 * data_vector[scale_elem->position-1]);
      break;
    case 1: /* linear scaling: y = a*x + b */
      new_value = scale_elem->parameter1 * data_vector[scale_elem->position-1] +
	scale_elem->parameter2;
      break;
    case 2: /* Binary scale if x<=param1 y=0; if x>= param2 y=1; else y = 0.5 */
      if (data_vector[scale_elem->position-1]<=scale_elem->parameter1)
	new_value = 0.0;
      else if (data_vector[scale_elem->position-1]>=scale_elem->parameter2)
	new_value = 1.0;
      else
	new_value = 0.5;
      break;
    case 3: /* Binary symmetric scale if x<=param1 y=-1; if x>= param2 y=1; else y = 0 */
      if (data_vector[scale_elem->position-1]<=scale_elem->parameter1)
	new_value = -1.0;
      else if (data_vector[scale_elem->position-1]>=scale_elem->parameter2)
	new_value = 1.0;
      else
	new_value = 0.;
      break;
    case 4: /* focus scaling : if x<a: y=s1*x if x>a:y=a*(s1-s2) + x*s2 */
      if(data_vector[scale_elem->position-1] > scale_elem->parameter1)
	new_value= scale_elem->parameter1*(scale_elem->parameter2-scale_elem->parameter3)
	  + scale_elem->parameter3 * data_vector[scale_elem->position-1];
      else if(data_vector[scale_elem->position-1] < - scale_elem->parameter1)
	new_value=-scale_elem->parameter1*(scale_elem->parameter2-scale_elem->parameter3)
	  + scale_elem->parameter3 * data_vector[scale_elem->position-1];
      else /* within focus */
	new_value= scale_elem->parameter2 * data_vector[scale_elem->position-1];
      break;
    } /* switch */
    data_vector[scale_elem->position-1] = new_value;
  } /* for */
}

void Net::apply_backward_scaling(double* data_vector, scale_typ *scale_list)
{ /* compute dout'/dout */

  scale_typ *scale_elem;
  double new_value = 0;

  for(scale_elem = scale_list; scale_elem != NULL; scale_elem = scale_elem->next){
    switch(scale_elem->scale_function){
    case 0: /* symmetric scaling : if x>=0: y=a*x if x<0:y=b*x */ 
      new_value=(data_vector[scale_elem->position-1]>=0?
		 scale_elem->parameter1 * data_vector[scale_elem->position-1]:
		 scale_elem->parameter2 * data_vector[scale_elem->position-1]);
      break;
    case 1: /* linear scaling: y = a*x + b */
      new_value = scale_elem->parameter1 * data_vector[scale_elem->position-1];
      break;
    case 2: 
	/* no gradient computation possible */
	new_value = data_vector[scale_elem->position-1];
	break;
    case 3: /* Binary symmetric scale if x<=param1 y=-1; if x>= param2 y=1; else y = 0 */
	/* no gradient computation possible */
	new_value = data_vector[scale_elem->position-1];
	break;
      break;
    } /* switch */
    data_vector[scale_elem->position-1] = new_value;
  } /* for */
}

/********************************************************************************/

/*       load/save network                                                      */

/********************************************************************************/

int Net::save_net( char filename[] )
{
  int i,l;
  weight_typ *wptr;
  scale_typ *scale_list;

  FILE *outfile=fopen(filename,"w");
  if (outfile==NULL){
    fprintf(stderr,"Output file %s cannot be opened\n", filename);
    return(FILE_ERROR);
  }

  fprintf(outfile,"# Layers: %d\n",topo_data.layer_count);
  fprintf(outfile,"# Topology (input - hidden - output)\n");
  fprintf(outfile,"topology: ");
  for(l= 0; l<topo_data.layer_count; l++)
    fprintf(outfile,"%d ",layer[l].unit_count);
  fprintf(outfile,"\nset_update_f %d ",update_id);
  for(i= 0; i<MAX_PARAMS; i++)
    fprintf(outfile,"%f ",update_params[i]);
  fprintf(outfile,"\n");
  if(network_type == RESERVOIR)
    fprintf(outfile,"Network type: RESERVOIR\n");
  for (scale_list = scale_list_in; scale_list != NULL; scale_list = scale_list->next)
    fprintf(outfile,"input_scale %d %d %g %g %g\n",scale_list->position,
	    scale_list->scale_function, scale_list->parameter1, scale_list->parameter2, 
	    scale_list->parameter3);
  fprintf(outfile,"\n");
  for (scale_list = scale_list_out; scale_list != NULL; scale_list = scale_list->next)
    fprintf(outfile,"output_scale %d %d %g %g %g\n",scale_list->position,
	    scale_list->scale_function, scale_list->parameter1, scale_list->parameter2,
	    scale_list->parameter3);
  fprintf(outfile,"\n\n");
  fprintf(outfile,"# Connection structure - format:\n");
  fprintf(outfile,"# <unit_no> <act_id>\n");
  fprintf(outfile,"# Weights <from_unit> <value>\n\n");
  for (l=1; l<topo_data.layer_count; l++)  /* start at first hidden layer */
    for (i=0; i<layer[l].unit_count; i++){
      fprintf(outfile,"%d %d\n",layer[l].unit[i].unit_no,layer[l].unit[i].act_id);
      for (wptr=layer[l].unit[i].weights; wptr!=NULL; wptr=wptr->next)
	fprintf(outfile,"%d %g  ",wptr->from_unit,wptr->value);
      fprintf(outfile,"\n");
    }
  fclose(outfile);
  return(OK);
}

#define MAX_LINE_LEN 10000 /* max. length of line in network description */

int Net::load_net( char filename[] )
{
  char secondchance[100], line[MAX_LINE_LEN];
  char *value;
  int read_in_topo[MAX_LAYERS];
  int i, current_unit, act_id, from_unit;
  FTYPE weight_value;
  int define_new;   /* type of network description */
  int mode,no,from;
  FTYPE range;
  double p[MAX_PARAMS];
  int position, scale_function;
  double para1, para2, para3;

  if(topo_data.unit_count){
    /* ridi 11/96: overwrite old net */
    // fprintf(stderr,"Net defined - OVERWRITING\n");
    delete_structure(); /* delete old net */
  }
  FILE *infile=fopen(filename,"r");
  if (infile==NULL){
    sprintf(secondchance,"%s.net",filename);
    if ((infile=fopen(secondchance,"r"))==NULL){
      fprintf(stderr,"Neither %s nor %s can be opened\n", filename, secondchance);
    return(FILE_ERROR);
    }
  }

  define_new = 0;   /* type of description not yet identified */
  while(fgets(line,MAX_LINE_LEN,infile)!=NULL){
    if(*line=='\n'||*line=='#');  /* skip comments */
    else if (strncmp(line,"topology",7)==0){
      value=strtok(line, " \t\n");  /* skip first token (== topology)*/
      for(i=0,value=strtok( NULL," \t" );(value!=NULL)&&(i<MAX_LAYERS);
	  value=strtok( NULL," \t\n" ),i++){
	read_in_topo[i] = (int)atoi(value);
      } /* finished reading topology  */
      if (create_layers(i,read_in_topo)) return (NPP_ERROR);
    }
    else if (strncmp(line,"set_update_f",12)==0){
      value=strtok(line, " \t\n");  /* skip first token (== set_update_f)*/
      value=strtok( NULL," \t\n" ); /* second token = type of update fun */
      mode = (int) atoi(value);
      for(i=0,value=strtok( NULL," \t" );(value!=NULL)&&(i<MAX_PARAMS);
	  value=strtok( NULL," \t\n" ),i++){
	p[i] = (double)atof(value);
      } /* finished reading update params  */
      set_update_f(mode,p);
    }
    else if (topo_data.unit_count){   /* units already defined */
      if (strncmp(line,"connect_layers",12)==0){
	connect_layers();
	define_new = 1;
      }
      else if (strncmp(line,"connect_shortcut",12)==0){
	connect_shortcut();
	define_new = 1;
      }
      else if (strncmp(line,"connect_reservoir",17)==0){
	value=strtok(line, " \t\n");  /* skip first token (== connect_shortcut)*/
	value=strtok( NULL," \t\n" ); /* second token = connectivity-rate */
	double connectivity_rate = (double) atof(value);
	printf("connectivity rate: %lf\n", connectivity_rate);
	connect_reservoir(connectivity_rate);
	define_new = 1;
      }
      else if (strncmp(line,"connect_units",12)==0){
	sscanf(line,"%*s %d %d",&no,&from);
	connect_units(no,from,range);
      }      
      else if (strncmp(line,"init_weights",10)==0){
	sscanf(line,"%*s %d %lf",&mode,&range);
	init_weights(mode,range);
      }
      else if (strncmp(line,"set_layer_act_f",12)==0){
	sscanf(line,"%*s %d %d",&no,&mode);
	set_layer_act_f(no,mode);
      }      
      else if (strncmp(line,"set_unit_act_f",12)==0){
	sscanf(line,"%*s %d %d",&no,&mode);
	set_unit_act_f(no,mode);
      }

      else if (strncmp(line,"Network type: RESERVOIR",23)==0){
	network_type = RESERVOIR;
	forward_pass_f = prop_forward_reservoir; // switch forward pass function to reservoir
      }
      else if (strncmp(line,"input_scale",10)==0){
	sscanf(line,"%*s %d %d %lf %lf %lf",&position,&scale_function,&para1,&para2,&para3);
	if ((position<1) || (position > topo_data.in_count)){
	  fprintf(stderr,"Error: Input_scale - undefined position %d\n",position);
	  return(NPP_ERROR);
	}
	insert_scale_element(&scale_list_in,position,scale_function,para1,para2,para3);
      }      
      else if (strncmp(line,"output_scale",10)==0){
	sscanf(line,"%*s %d %d %lf %lf %lf",&position,&scale_function,&para1,&para2,&para3);
	if ((position<1) || (position > topo_data.out_count)){
	  fprintf(stderr,"Error: Output_scale - undefined position %d\n",position);
	  return(NPP_ERROR);
	}
	insert_scale_element(&scale_list_out,position,scale_function,para1,para2,para3);
      }      
      else if (! define_new){ /* read  weights from file */      
	/* read current unit_no and activation function identity */
	sscanf(line,"%d %d",&current_unit, &act_id);
	set_unit_act_f(current_unit, act_id);
	/* read weights (in next line) */
        char* cres = fgets(line,MAX_LINE_LEN,infile);
        if (cres==NULL) fprintf(stderr,"Error: reached end of file without sucess or general error in fgets");
	for(value=strtok( line," \t" );value!=NULL && *value!='\n';
	    value=strtok( NULL," \t" )){
	  from_unit = (int) atoi(value);
	  value=strtok( NULL," \t" );
	  weight_value = (FTYPE) atof(value);
	  connect_units(current_unit,from_unit,weight_value);
	} /* for all weights */
      }  /* if read weights from file */
    } /* if units already defined */
  } /* while read line from file */
  // 30.11.2011 A final(?) bug. This must be again 
  // called after weight structure to initialize Rprop or BP 
  set_update_f(update_id,update_params); 
  fclose(infile);  /* Martin, wie konntest du das vergessen ???? */
  return(OK);
}

void Net::print_net()
{
  int i,l;
  weight_typ *wptr;

  printf("# Layers:\n%d\n",topo_data.layer_count);
  printf("# Topology (input - hidden - output)\n");
  for(l= 0; l<topo_data.layer_count; l++)
    printf("%d ",layer[l].unit_count);
  printf("\n");
  printf("Update Function: %d\nCurrent update Params: ",update_id);
  for(i=0;i<MAX_PARAMS;i++)
    printf("%f  ",update_params[i]);
  printf("\n\n");
  printf("# Weights:\n");
  for (l=0; l<topo_data.layer_count; l++)
    for (i=0; i<layer[l].unit_count; i++){
      printf("unit %d) act_id: %d   net_in:  %g   out %g\n",
	     layer[l].unit[i].unit_no,layer[l].unit[i].act_id,
	     layer[l].unit[i].net_in,layer[l].unit[i].out);
      for (wptr=layer[l].unit[i].weights; wptr!=NULL; wptr=wptr->next)
	printf("%d %g  ",wptr->from_unit,wptr->value);
      printf("\n");
    }
}

void Net::print_net_activations()
{
  int i,l;

  for (l=0; l<topo_data.layer_count; l++)
    for (i=0; i<layer[l].unit_count; i++){
      printf("unit %d) act_id: %d   net_in:  %g   out %g\n",
	     layer[l].unit[i].unit_no,layer[l].unit[i].act_id,
	     layer[l].unit[i].net_in,layer[l].unit[i].out);
    }
}

/********************************************************************************/

/*       set/change network functions                                           */

/********************************************************************************/

void Net::set_unit_act_f(int unit_no,int act_id){
/* set activation function of unit unit_no */
  if(!IS_UNIT(unit_no)){
    fprintf(stderr,"set act_f: Unit %d doesn't exists\n",unit_no);
    return;
  }
  if(IS_INPUT(unit_no)){
    fprintf(stderr,"set act_f: No sense to set activation function of input unit %d\n",unit_no);
    return;
  }
  switch(act_id){
  case LOGISTIC:
    unit[unit_no].act_f = logistic;
    unit[unit_no].deriv_f = logistic_deriv;
    unit[unit_no].act_id = LOGISTIC;
    break;
  case SYMMETRIC:
    unit[unit_no].act_f = symmetric;
    unit[unit_no].deriv_f = symmetric_deriv;
    unit[unit_no].act_id = SYMMETRIC;
    break;
  case LINEAR:
    unit[unit_no].act_f = linear;
    unit[unit_no].deriv_f = linear_deriv;
    unit[unit_no].act_id = LINEAR;
    break;
  default:
    fprintf(stderr,"set act_f: Unknown activation function %d\n",act_id);
  }
}

void Net::set_layer_act_f(int layer_no,int act_id){
/* set activation function of a complete layer */
  int i;

  if(!IS_LAYER(layer_no)){
    fprintf(stderr,"set_layer_act_f: Layer %d doesn't exist\n",layer_no);
    return;
  }
  for (i=0; i<layer[layer_no].unit_count; i++){
    set_unit_act_f(layer[layer_no].unit[i].unit_no,act_id);
  }
}

int Net::set_update_f(int typ, double* params)
/* set current update function, init update_function, set update_params */
{
  int i;

  switch(typ){
  case BP:
    bp_init(params,unit,&topo_data);  /* prepare bp, set update_params */
    update_f = bp_update;
    update_id = BP;
    break;
  case RPROP:
    rprop_init(params,unit,&topo_data); /* prepare rprop */
    update_f = rprop_update;
    update_id = RPROP;
    break;
  default:
    fprintf(stderr,"set update_f: Unknown update function %d\n",typ);
    return (NPP_ERROR);
  }
  /* copy to internal update_params */
  for(i=0;i<MAX_PARAMS;i++)
    update_params[i] = params[i];
  return(0);
}

/********************************************************************************/

/*       working procedures                                                     */

/********************************************************************************/

void Net::set_seed(long seed_no)
{
  srand48(seed_no);
}

void Net::reset_reservoir_neurons(){
  for(int i=topo_data.in_count+1;i<=topo_data.unit_count;i++){
    unit[i].out = unit[i].new_out = (FTYPE) 0;
  }
}

void Net::init_weights(int mode, FTYPE range)
/* init the weights */
{
  int i;
  weight_typ *wptr;

  if(mode == 0){
    for(i=topo_data.in_count+1;i<=topo_data.unit_count;i++) /* for all except inputs */
      for (wptr=unit[i].weights; wptr!=NULL; wptr=wptr->next)
	wptr->value = (FTYPE)((2.0 * range * drand48()) -range);
  }
  else if(mode == 1){ /* all biases = 0.0 */
    for(i=topo_data.in_count+1;i<=topo_data.unit_count;i++){ /* for all except inputs */
      if((wptr=unit[i].weights)!=NULL){
	wptr->value = (FTYPE) 0;
	wptr=wptr->next;
	for (; wptr!=NULL; wptr=wptr->next)
	  wptr->value = (FTYPE)((2.0 * range * drand48()) -range);
      }
    }
  }
  else if(mode == 2){
    for(i=topo_data.in_count+1;i<=topo_data.unit_count;i++) /* for all except inputs */
      for (wptr=unit[i].weights; wptr!=NULL; wptr=wptr->next){
	if (wptr->from_unit == i)
	  wptr->value = 1.0;
	else
	  wptr->value = (FTYPE)((2.0 * range * drand48()) -range);
      }
  }

}

void Net::forward_pass_encoder(FTYPE *in_vec, FTYPE *out_vec){
  //  printf("Forward pass encoder\n");

  int first_encoder_unit = layer[topo_data.encoder_layer].unit[0].unit_no;

  if (scale_list_in != NULL){
    for(int i=0;i<topo_data.in_count;i++)
      scaled_in_vec[i] = in_vec[i];
    apply_scaling(scaled_in_vec,scale_list_in);
    for(int i=0;i<topo_data.in_count;i++)
      unit[i+1].out = scaled_in_vec[i];      
  }
  else{
    for(int i=0;i<topo_data.in_count;i++)
      unit[i+1].out = in_vec[i];      
  }

  int i;
  weight_typ *wptr;
  
  for(i=topo_data.in_count+1;i<=first_encoder_unit + topo_data.encoder_count;i++){
    unit[i].net_in = (FTYPE) 0;  /* reset net in */
    for (wptr=unit[i].weights; wptr!=NULL; wptr=wptr->next) /* compute new net in */
      unit[i].net_in += wptr->value* unit[wptr->from_unit].out;
    unit[i].out = unit[i].act_f(&unit[i]); /* compute unit activation and output */
  }

  for(i=0;i<topo_data.encoder_count;i++)
    out_vec[i] = layer[topo_data.encoder_layer].unit[i].out;  
  //  print_network();
}


void Net::forward_pass(FTYPE *in_vec, FTYPE *out_vec)
/* take input vector, pass through network, return output vector */
{
  int i;
  
  if (scale_list_in != NULL){
    for(i=0;i<topo_data.in_count;i++)
      scaled_in_vec[i] = in_vec[i];
    apply_scaling(scaled_in_vec,scale_list_in);
    for(i=0;i<topo_data.in_count;i++)
      unit[i+1].out = scaled_in_vec[i];      
  }
  else{
    for(i=0;i<topo_data.in_count;i++)
      unit[i+1].out = in_vec[i];      
  }
  forward_pass_f(unit,&topo_data); /* call (user specified) forward pass function */
  for(i=0;i<topo_data.out_count;i++)
    out_vec[i] = unit[(topo_data.unit_count-topo_data.out_count+1)+i].out; 
  if (scale_list_out != NULL)
    apply_scaling(out_vec,scale_list_out);
}

void Net::backward_pass(FTYPE *dedout, FTYPE *dedin)
/* take vector de/dout, backward pass, compute de/dw and de/dinput */
{
  int i;
  
  if (scale_list_out != NULL)
    apply_backward_scaling(dedout,scale_list_out);
  for(i=0;i<topo_data.out_count;i++)
    unit[(topo_data.unit_count-topo_data.out_count+1)+i].dEdo = dedout[i];      
  backward_pass_f(unit,&topo_data); /* call (user specified) backward pass function */
  for(i=0;i<topo_data.in_count;i++)
    dedin[i] = unit[i+1].dEdnet; 
  if (scale_list_in != NULL)
    apply_backward_scaling(dedin,scale_list_in);
}

void Net::backward_pass_light(FTYPE *dedout, FTYPE *dedin)
/* take vector de/dout, backward pass, compute de/dinput
SPECIAL: do not compute dEdw !!! (only dE/din) */
{
  int i;
  
  if (scale_list_out != NULL)
    apply_backward_scaling(dedout,scale_list_out);
  for(i=0;i<topo_data.out_count;i++)
    unit[(topo_data.unit_count-topo_data.out_count+1)+i].dEdo = dedout[i];      
  do_backward_pass_light(unit,&topo_data); /* call light backward pass function */
  for(i=0;i<topo_data.in_count;i++)
    dedin[i] = unit[i+1].dEdnet; 
  if (scale_list_in != NULL)
    apply_backward_scaling(dedin,scale_list_in);
}

void Net::update_weights()
/* call current update function to adapt the weights */
{
  update_f(unit,&topo_data,update_params);
}


/********************************************************************************/

/*       TD procedures                                                          */

/********************************************************************************/

void Net::TD_backward_pass(FTYPE td_error, double lambda)
/* computes dEdw_sequence 
   according to TD learning rule
   different meanings:
   dEdo == dO/do_i;       (influence of output i on network output O)
   dEdnet == dO/dnet_i;   (...)
   dEdw_sequence = summed error of sequence; must be stored by explicit call
                   of TD_terminate_sequence(weighting factor)
*/
{  
  int i;
  weight_typ *wptr;

  unit[topo_data.unit_count].dEdo = (FTYPE) 1; /* for output: dO/do_i = 1 */
  for(i=topo_data.unit_count;i>topo_data.in_count;i--){
    unit[i].dEdnet = unit[i].dEdo * unit[i].deriv_f(&unit[i]); /* dO/dnet_i */
    unit[i].dEdo = (FTYPE) 0;  /* clear for next time */
    for (wptr=unit[i].weights; wptr!=NULL; wptr=wptr->next){
      wptr->dEdw_sequence += wptr->dOdw * td_error;
      wptr->dOdw = lambda * wptr->dOdw + unit[i].dEdnet * unit[wptr->from_unit].out; 
      unit[wptr->from_unit].dEdo += unit[i].dEdnet * wptr->value; /* de/do_j */
    }
  }
}

void Net::TD_init_sequence()
{
  int i;
  weight_typ *wptr;

  for(i=topo_data.unit_count;i>0;i--){
    unit[i].dEdnet = unit[i].dEdo = (FTYPE) 0;  /* clear for next time */
    for (wptr=unit[i].weights; wptr!=NULL; wptr=wptr->next){
      wptr->dOdw = (FTYPE) 0;  /* clear for next time */
      wptr->dEdw_sequence = (FTYPE) 0;  /* clear for next time */
    }
  }
}

void Net::TD_terminate_sequence(double weighting_factor)
/* add sequence to dEdw, weighted by a weighting_factor */
{
  int i;
  weight_typ *wptr;

  for(i=topo_data.unit_count;i>0;i--){
    for (wptr=unit[i].weights; wptr!=NULL; wptr=wptr->next){
      wptr->dEdw += weighting_factor * wptr->dEdw_sequence;
    }
  }
}

void Net::clear_derivatives()
/* special: set all derivatives to zero; 
   used e.g. for useless sequences in TD-learning */
{
  int i;
  weight_typ *wptr;

  for(i=topo_data.unit_count;i>0;i--){
    for (wptr=unit[i].weights; wptr!=NULL; wptr=wptr->next){
      wptr->dEdw = (FTYPE) 0;  /* clear for next time */
    }
  }
}

void Net::multiply_derivatives(double factor)
/* special: multiply all derivatives with factor; 
   used e.g. to stress sequences in TD-learning */
{
  int i;
  weight_typ *wptr;

  for(i=topo_data.unit_count;i>0;i--){
    for (wptr=unit[i].weights; wptr!=NULL; wptr=wptr->next){
      wptr->dEdw *= factor;  /* clear for next time */
    }
  }
}
