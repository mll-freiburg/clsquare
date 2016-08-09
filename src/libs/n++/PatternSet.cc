/******************************************************************************/

/* N++ : C++ class neural network simulator                                   */
/* (c) 1994 Martin Riedmiller                                                 */
/* last changed: 27.7.94                                                      */

/* File: PatternSet.cc                                                        */
/*       management (load, store) of patternfiles                             */

/******************************************************************************/

#include "PatternSet.h"


PatternSet::PatternSet(void)
{
  pattern_count=0;
  input_count = target_count = 0;
  max_pattern_count = 0;
  name = NULL;
  input = NULL;
  target = NULL;
}

void PatternSet::create(long size, int insize, int targetsize){
  if(max_pattern_count >0)
    return;
  input_count = insize;
  target_count = targetsize;
  max_pattern_count = size;
  pattern_count=0;
  input = new double* [size];
  target = new double* [size];  
}

void PatternSet::make_clean(){
  long i;
  if (name){
    for(i=0;i < pattern_count;i++)
      if (name[i]) delete name[i];
  }
  if (input){
    for(i=0;i< pattern_count;i++)
      delete[] input[i];
  }
  if (target){
    for(i=0;i< pattern_count;i++)
      delete[] target[i];
  }
  pattern_count=0;
}

void PatternSet::add_pattern(const double *in, const double *out){
  if(pattern_count >= max_pattern_count) {
    fprintf(stderr,"Can't add pattern, maximum size of %ld reached",max_pattern_count);
    return;
  }
  input[pattern_count] = new double [input_count];
  target[pattern_count] = new double [target_count];
  for(int i=0;i<input_count;i++)
    input[pattern_count][i] = in[i];
  for(int i=0;i<target_count;i++)
    target[pattern_count][i] = out[i];
  pattern_count ++;
}


int PatternSet::load_pattern(char *filename)
{
  FILE *patf;
  char secondchance[MAX_STRING_LEN],s[MAX_STRING_LEN],part[MAX_STRING_LEN];
  char *value;
  int expecting;
  int i,j;
  long p;

  if((patf=fopen(filename,"r")) == NULL){
    sprintf(secondchance,"%s.pat",filename);  /* try with extension .pat */
    if((patf=fopen(secondchance,"r")) == NULL){
      fprintf(stderr,"Can't open patternfile neither %s nor %s\n",filename,secondchance);
      return (PAT_FILE_ERROR);
    }
  }

  /* Allocate pointer arrays of length max_no_of_pattern */
  name = new char* [MAX_NO_OF_PATTERN];
  if (name == 0){
    printf("Kein Speicherplatz bei Patternset!\n");
    exit(1);
  }
  input = new double* [MAX_NO_OF_PATTERN];
  if (input == 0){
    printf("Kein Speicherplatz bei Patternset!\n");
    exit(1);
  }
  target = new double* [MAX_NO_OF_PATTERN];
  if (target == 0){
    printf("Kein Speicherplatz bei Patternset!\n");
    exit(1);
  }

  expecting = 3;
  while ((expecting) && (fgets (s,MAX_STRING_LEN,patf) != NULL)){ 
    if (strncmp (s,"No.",2) == 0){
      sscanf(s,"%*s %*s %s",part);
      if (strncmp (part,"patterns",6) == 0){
	sscanf(s,"%*s %*s %*s %*s %s",part);
	/* pattern_count = atoi(part);  throw away information - don't trust */
	expecting --;
      }
      if (strncmp (part,"input",5) == 0){
	sscanf(s,"%*s %*s %*s %*s %*s %s",part);
	input_count = atoi(part);
	expecting --;
      }
      if (strncmp (part,"output",5) == 0){
	sscanf(s,"%*s %*s %*s %*s %*s %s",part);
	target_count = atoi(part);
	expecting --;
      }
    }	
  } /* end of reading header */

  p=0;
  while((fgets (s,MAX_STRING_LEN,patf) != NULL) && (p<MAX_NO_OF_PATTERN)){ 
    if(*s =='%'||*s ==' '||*s =='\n'); /* skip comments and empty lines */
    else{ /* new pattern read, allocate arrays */
      if (*s == '#'){ /* pattern has a patternname */
	name[p] = new char[LENPATNAME];
	if (name[p] == 0){
	  printf("Kein Speicherplatz bei Patternset!\n");
	  exit(1);
	}
	j=0;
	while(((name[p][j]=s[j])!=0) && j<LENPATNAME-1) j++;
	name[p][j] = 0;
      }
      else { /* read input and target */
	input[p] = new double [input_count];
	if (input[p] == 0){
	  printf("Kein Speicherplatz bei Patternset!\n");
	  exit(1);
	}
	target[p] = new double [target_count];
	if (target[p] == 0){
	  printf("Kein Speicherplatz bei Patternset!\n");
	  exit(1);
	}
	for(i=0,value=strtok(s, " \t");
	    (value!=NULL)&&i<input_count;value=strtok( NULL," \t" ),i++){
	  /* parse input line */
	  input[p][i] = (double)atof(value);
	} /* for */
	if (target_count>0){  /* read target pattern */
	  do{
            char* cres = fgets (s,MAX_STRING_LEN,patf);
            if (cres==NULL) fprintf(stderr,"Error: reached end of file without sucess or general error in fgets");
	  }
	  while ((*s =='%') || (*s =='#') || (*s ==' ') || (*s =='\n')); 
	  /* skip comments and empty lines */
	  for(i=0,value=strtok(s, " \t");
	      (value!=NULL)&&i<target_count;value=strtok( NULL," \t" ),i++){
	    /* parse target line */
	    target[p][i] = (double)atof(value);
	  }/* for */
	} /* if targets */
	p++;
      } /* else input/target pattern read */
    } /* end of else reading pattern */
  } /* while there's data in pattern file */
  pattern_count = p;
  fclose(patf);  /* Martin, wie konntest du das vergessen ???? */
  return(PAT_OK);
}

PatternSet::~PatternSet()
{
  long i;
  if (name){
    for(i=0;i < pattern_count;i++)
      if (name[i]) delete name[i];
    delete[] name;
  }
  if (input){
    for(i=0;i< pattern_count;i++)
      delete[] input[i];
    delete[] input;
  }
  if (target){
    for(i=0;i< pattern_count;i++)
      delete[] target[i];
    delete[] target;
  }
}

void PatternSet::print_pattern()
{
  int i;
  long p;

  printf("no patterns %ld\nno inputs %d\nno outputs %d\n\n",
	 pattern_count,input_count,target_count);
  for(p=0;p<pattern_count;p++){
    if (name && name[p])
      printf("%s\n", name[p]);
    for(i=0;i<input_count;i++)
      printf("%5.6f  ",input[p][i]);
    printf("\n");
    for(i=0;i<target_count;i++)
      printf("%5.6f  ",target[p][i]);
    printf("\n\n");
  }
    
}
