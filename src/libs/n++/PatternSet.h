/******************************************************************************/

/* N++ : C++ class neural network simulator                                   */
/* (c) 1994 Martin Riedmiller                                                 */
/* last changed: 27.7.94                                                      */

/* File: PatternSet.h                                                         */
/* Purpose: PatternSet header file - to be included in application programs   */

/******************************************************************************/

#include<stdio.h>
#include<string.h>
#include<stdlib.h>

#define PAT_OK 0
#define PAT_FILE_ERROR -1
#define LENPATNAME 30
#define MAX_STRING_LEN 5000
#define MAX_NO_OF_PATTERN 10000000

class PatternSet{
 protected:
 public:
  long pattern_count;
  long max_pattern_count;
  int input_count,target_count;
  char **name;
  double **input,**target;
  PatternSet(void);
  virtual ~PatternSet();
  virtual void make_clean();
  virtual int load_pattern(char *filename);
  virtual  void print_pattern();
  virtual void create(long size, int insize, int targetsize);
  virtual void add_pattern(const double *in, const double *target);
};



