#define ONLINE 1
#define OFFLINE 0

struct result_type{
  double tss;             
  int   hamdis;    /* hamming distance: number of wrong classified outputs */
};

struct param_type{
  double tolerance;     /* error tolerance for classification error */
  int update_mode;     
}; 

extern param_type aux_params;

void aux_trainAll(Net *net,PatternSet *pat,result_type *result);
void aux_testAll(Net *net,PatternSet *pat,result_type *result);
void aux_testAllVerbose(Net *net,PatternSet *pat,result_type *result);

