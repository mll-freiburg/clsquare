/******************************************************************************/

/* aux.c               
   providing auxillary functions and procedures to perform
   training and testing of patternfiles
   using N++ and PatternSet
*/

/******************************************************************************/

#include <stdio.h>
#ifdef MSDOS
#include "npp.h"
#else
#include "n++.h"
#endif
#include "PatternSet.h"
#include "aux.h"

#define ABS(x) (x>0?x:-x)

param_type aux_params = {0.0,OFFLINE};

void aux_trainAll(Net *net,PatternSet *pat,result_type *result)
/* train all pattern of pattern set pat */
{
  register double error,tss;  /* total square sum */
  int p,i;
  int hamdis = 0;
  register int online;

  online = aux_params.update_mode;
  tss = 0.0;
  for(p=0;p<pat->pattern_count;p++){
    net->forward_pass(pat->input[p],net->out_vec);
    for(i=0;i<net->topo_data.out_count;i++){
      /* out_vec = dE/do = (o-t) */
      error = net->out_vec[i] = net->out_vec[i] - pat->target[p][i]; 
      if (ABS(error) > aux_params.tolerance) hamdis ++;
      tss += error * error;
    }
    /*
    printf("aux.cc train: dEdo: ");
    for(i=0;i<net->topo_data.out_count;i++)
      printf(" %.3f ",net->out_vec[i]);
    printf("\n");
    */

    net->backward_pass(net->out_vec,net->in_vec);
    if(online) net->update_weights();  /* learn by pattern */
  }
  if(!online)  net->update_weights();  /* learn by epoch */
  result->tss = tss;
  result->hamdis = hamdis;
}

void aux_testAll(Net *net,PatternSet *pat,result_type *result)
{
  double error,tss;  /* total square sum */
  int p,i;
  int hamdis = 0;
  
  tss = 0.0;
  for(p=0;p<pat->pattern_count;p++){
    net->forward_pass(pat->input[p],net->out_vec);
    for(i=0;i<net->topo_data.out_count;i++){
      error = net->out_vec[i] - pat->target[p][i]; /* out_vec = dE/do = (o-t) */
      if (ABS(error) > aux_params.tolerance) hamdis ++;
      tss += error * error;
    }
  }
  result->tss = tss;
  result->hamdis = hamdis;
}

void aux_testAllVerbose(Net *net,PatternSet *pat,result_type *result)
{
  double error,tss;  /* total square sum */
  int p,i;
  int hamdis = 0;
  
  tss = 0.0;
  for(p=0;p<pat->pattern_count;p++){
    net->forward_pass(pat->input[p],net->out_vec);
    printf("in:   ");
    for(i=0;i<pat->input_count;i++)
      printf("%5.3f ",pat->input[p][i]);
    printf("\nout:  ");
    for(i=0;i<net->topo_data.out_count;i++){
      printf("%5.3f ",net->out_vec[i]);
      error = net->out_vec[i] - pat->target[p][i]; /* out_vec = dE/do = (o-t) */
      if (ABS(error) > aux_params.tolerance) hamdis ++;
      tss += error * error;
    }
    printf("\ntarg: ");
    for(i=0;i<pat->target_count;i++)
      printf("%5.3f ",pat->target[p][i]);
    printf("\n\n");
  }
  printf("%f\n\n",tss);
  result->tss = tss;
  result->hamdis = hamdis;
}








