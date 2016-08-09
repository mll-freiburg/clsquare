#include <stdio.h>
#include <iostream>
#include <math.h>
#include <string.h>
#include <stdlib.h>
#include <iomanip>

#include <dirent.h>


using namespace std;

#define MAX_EPISODES 5000
#define MAX_ENTRIES 20
#define MAX_LINE_LEN 5000
#define MAX_NETNAME 1000
#define CYCLES_IN_XPLUS 6
#define CYCLES_OUTOF_XPLUS 5
#define FINAL_REWARD 9
#define TERMINATED_IN_XPLUS 2
#define TOUCHED_XPLUS 1

double deltat = 0.01;

struct seval_stat {
  double data[MAX_EPISODES][MAX_ENTRIES];  
  long num_episodes;
  char netname[MAX_EPISODES][MAX_NETNAME];
  long sorted_by_costs[MAX_EPISODES];
} statistics;


double get_cycles_in_xplus(long from_episode, long to_episode){

  double result = 0.;
  for(long i=from_episode; i< to_episode; i++){
    result += statistics.data[i][CYCLES_IN_XPLUS];
  }
  return result;
}

double get_avg_time_outof_xplus(long from_episode, long to_episode){

  double result = 0.;
  for(long i=from_episode; i< to_episode; i++){
    result += statistics.data[i][CYCLES_OUTOF_XPLUS];
  }
  return result*deltat/(double)(to_episode-from_episode);
}

long get_first_successful_controller(long from_episode, long to_episode){

  for(long i=from_episode; i< to_episode; i++){
    if(statistics.data[i][TERMINATED_IN_XPLUS] == 100.0)
      return i;
  }
  return -1;
}

long get_first_touched_controller(long from_episode, long to_episode){

  for(long i=from_episode; i< to_episode; i++){
    if(statistics.data[i][TOUCHED_XPLUS] == 100.0)
      return i;
  }
  return -1;
}

double get_avg_terminated_all_in_xplus(long from_episode, long to_episode){
  double number = 0;
  
  for(long i=from_episode; i< to_episode; i++){
    if(statistics.data[i][TERMINATED_IN_XPLUS] == 100.0)
      number ++;
  }
  return 100.0 * number/(double)(to_episode-from_episode);
}

double get_avg_terminated_in_xplus(long from_episode, long to_episode){
  double result = 0;
  
  for(long i=from_episode; i< to_episode; i++){
    result += statistics.data[i][TERMINATED_IN_XPLUS];
  }

  return result/(double)(to_episode-from_episode);
}

double get_avg_touched_xplus(long from_episode, long to_episode){
  double result = 0;
  
  for(long i=from_episode; i< to_episode; i++){
    result += statistics.data[i][TOUCHED_XPLUS];
  }

  return result/(double)(to_episode-from_episode);
}

long get_best_controller(long from_episode, long to_episode){

  long best_idx = -1;
  double best_value = 1E6;

  for(long i=from_episode; i< to_episode; i++){
    if(statistics.data[i][TERMINATED_IN_XPLUS] == 100.0){
      if(statistics.data[i][CYCLES_OUTOF_XPLUS] < best_value){
	best_value = statistics.data[i][CYCLES_OUTOF_XPLUS];
	best_idx = i;
      }
    }
  }
  return best_idx;
}

void sort_by_costs(long from_episode, long to_episode){

  for(long i=from_episode; i< to_episode; i++){
    statistics.sorted_by_costs[i] = -1; // init
  }

  long my_idx;
  for(long i=from_episode; i< to_episode; i++){
    if(statistics.data[i][TERMINATED_IN_XPLUS] == 100.0){
      for(long j=0;j<i+1;j++){
	if(statistics.sorted_by_costs[j] == -1){
	  //	  cout<<"endo of list reached"<<endl;
	  statistics.sorted_by_costs[j] = i;
	  break;
	}
	if(statistics.data[i][CYCLES_OUTOF_XPLUS]<
	   statistics.data[statistics.sorted_by_costs[j]][CYCLES_OUTOF_XPLUS]){
	  // this is MY place!
	  for(long k=i+1;k>j;k--) // make place
	    statistics.sorted_by_costs[k]=statistics.sorted_by_costs[k-1];
	  statistics.sorted_by_costs[j] = i;
	  break;
	}
      }
      
    }
  }
}



long get_most_precise_controller(long from_episode, long to_episode){

  long best_idx = -1;
  double best_value = 1E6;
  double best_precision = 1E6;

  for(long i=from_episode; i< to_episode; i++){
    if(statistics.data[i][TERMINATED_IN_XPLUS] == 100.0){
      if(statistics.data[i][FINAL_REWARD] < best_precision){
	best_precision = statistics.data[i][FINAL_REWARD];
	best_value = statistics.data[i][CYCLES_OUTOF_XPLUS];
	best_idx = i;
      }
      else if(statistics.data[i][FINAL_REWARD] == best_precision){
	//	cout<<"Equal precision at episode "<<i<<endl;
	if(statistics.data[i][CYCLES_OUTOF_XPLUS] < best_value){
	  best_precision = statistics.data[i][FINAL_REWARD];
	  best_value = statistics.data[i][CYCLES_OUTOF_XPLUS];
	  best_idx = i;
	  //	  cout<<"Equal precision at episode "<<i<<" better performance"<<endl;
	}
      }
    }
  }
  return best_idx;
}

void print_latex_statistics_of_controller(long episode_ctr){
  double outofxp= statistics.data[episode_ctr][CYCLES_OUTOF_XPLUS];
  double time_outofxp = outofxp * deltat;
    
  if(episode_ctr<0){
    cout<<" - &&& & & \\\\"<<endl;
    return;
  }
  
  cout<<episode_ctr<<"  & " ;

  double best_outofxp = 
    statistics.data[get_best_controller(0,statistics.num_episodes)][CYCLES_OUTOF_XPLUS];
  cout<<outofxp<<" & "<< setprecision(3)<< time_outofxp <<"s & "<< setprecision(-1);

  if(best_outofxp>0){
    cout<< setprecision(3)<< outofxp/ best_outofxp <<" & "<< setprecision(-1);
  }
  else 
    cout<<" - & ";
  cout<< statistics.data[episode_ctr][FINAL_REWARD]<<"m \\\\";
}


void print_statistics_of_controller(long episode_ctr){

  if(episode_ctr<0){
    cout<<"No controller found"<<endl;
    return;
  }

  cout<<"Episode "<<episode_ctr<<", netname: "
      <<statistics.netname[episode_ctr]<<" (";
  for(int entry_ctr =0; entry_ctr<10; entry_ctr++){
    cout<<statistics.data[episode_ctr][entry_ctr]<<" ";
  }
  cout<<")"<<endl;
  cout<<"Term. in X+: "<<statistics.data[episode_ctr][TERMINATED_IN_XPLUS]
      <<" || cycles out of X+ "<<statistics.data[episode_ctr][CYCLES_OUTOF_XPLUS]
      <<" || cycles in X+ "<<statistics.data[episode_ctr][CYCLES_IN_XPLUS]
      <<" || final reward: "<<statistics.data[episode_ctr][FINAL_REWARD]<<endl<<endl;
}

void print_statistics_table(){
  for(int episode_ctr =0; episode_ctr<statistics.num_episodes; episode_ctr++){
    print_statistics_of_controller(episode_ctr);
  }
  cout<<endl;
}

void read_file(char *filename){
  //  cout<<"Reading file"<<filename<<endl;

  char line[MAX_LINE_LEN];
  char *value;
  FILE *infile=fopen(filename,"r");
  if (infile==NULL){
    cout<<"Can't open file "<<filename<<". Exiting."<<endl;
    exit(0);
  }

  int episode_ctr = 0;
  int entry_ctr;
  while(fgets(line,MAX_LINE_LEN,infile)!=NULL){
    value=strtok( line," \t" );    
    if(*value=='\n'){
      //      cout<<"skip empty line :"<<line<<endl;
    }
    else if(*value=='#'){
      value=strtok( NULL," \t" );    
      //      cout<<" comment line second token "<<value;
      if (strncmp(value,"Testing",7)==0){
	value=strtok( NULL," \t\n" );    
	value=strtok( NULL," \t\n" );    
	//	cout<<" testing net "<<value;
	sprintf(statistics.netname[episode_ctr],"%s", value);
      }
      //      cout<<" - skip comment line :"<<line<<endl;
    }  /* skip comments */
    else{
      //      cout<<" complete line :"<<line<<endl;
      entry_ctr=0;
      statistics.data[episode_ctr][entry_ctr] = (double)atof(value);
      entry_ctr ++;
      for(value=strtok( NULL," \t\n" );(value!=NULL)&&
	    (entry_ctr<MAX_ENTRIES);
	  value=strtok( NULL," \t\n" ),entry_ctr++){
	statistics.data[episode_ctr][entry_ctr] = (double)atof(value);
      } /* finished reading line  */
      episode_ctr ++;
    }
  }
  statistics.num_episodes= episode_ctr;
}


#if 1
void print_statistics(int modus){
  long idx = get_first_successful_controller(0,statistics.num_episodes);
  if(modus == 0){
    cout<<"First succesful controller at episode "<<idx<<endl;
    print_statistics_of_controller(idx);
  }
  idx = get_best_controller(0,statistics.num_episodes);
  cout<<"Best controller at episode "<<idx<<endl;
  print_statistics_of_controller(idx);
  if(modus == 0){
    idx = get_most_precise_controller(0,statistics.num_episodes);
    cout<<"Most precise controller at episode "<<idx<<endl;
    print_statistics_of_controller(idx);
  }
}

#endif

void print_latex_statistics(int modus){
  // SPEED AND QUALITY OF LEARNING
  if(modus >0)
    return;

  cout<<"Learning speed and quality over "<<statistics.num_episodes<<" test runs.\\\\[.5em]"<<endl;
  cout<<"\\begin{tabular}{c|c|c|c|c|c}"<<endl;
  cout<<"Controller          &  episode &  cycles & realtime & perf. loss & precision  \\\\ \\hline"<<endl;
  long idx = get_first_touched_controller(0,statistics.num_episodes);
  cout<<"First touched 100\\% $X^+$ & ";
  print_latex_statistics_of_controller(idx);
  cout << endl;
  idx = get_first_successful_controller(0,statistics.num_episodes);
  cout<<"First 100\\% success & ";
  print_latex_statistics_of_controller(idx);
  cout << endl;
  idx = get_best_controller(0,statistics.num_episodes);
  cout<<"Best 100 \\% success  & ";
  print_latex_statistics_of_controller(idx);
  cout << endl;
  idx = get_most_precise_controller(0,statistics.num_episodes);
  cout<<"Most precise  & ";
  print_latex_statistics_of_controller(idx);
  cout << endl;

  cout<<"\\end{tabular}"<<endl<<endl;


  sort_by_costs(0,statistics.num_episodes);

  cout<<"List of best controllers\\\\[.5em]"<<endl;

  cout<<"\\begin{tabular}{c|c|c|c|c|c}"<<endl;
  cout<<"Controller          &  episode &  cycles & realtime & perf. loss & precision  \\\\ \\hline"<<endl;

  for(int i=0;i<statistics.num_episodes && i<10;i++){
    if(statistics.sorted_by_costs[i]>-1){
      cout<<i+1<<". (of "<<statistics.num_episodes <<") & ";
      print_latex_statistics_of_controller(statistics.sorted_by_costs[i]);
      cout << endl;
    }
  }
  cout<<"\\end{tabular}"<<endl<<endl;


  // STABILITY OF LEARNING
  cout<<endl<<"Stability of learning\\\\[.5em]"<<endl;
  cout<<"\\begin{tiny}"<<endl;
  cout<<"\\begin{tabular}{c|c|c|c|c|c|c|c|c|c|cc}"<<endl;
  cout<<"Episodes & ";
  for(int from =0; from < statistics.num_episodes; from += 100){
    cout<<" - "<<from +100<<" & "; 
  }
  cout<<"\\\\ \\hline" <<endl;

  cout<<"Avg. time out $X^+$ & ";
  cout<< setprecision(3);
  for(int from =0; from < statistics.num_episodes; from += 100){
    cout<<get_avg_time_outof_xplus(from, from+100)<<"s & ";
  }
  cout<< setprecision(-1);
  cout<<"\\\\ \\hline" <<endl;

  cout<<"Runs term. in $X^+$ & ";
  cout<< setprecision(3);
  for(int from =0; from < statistics.num_episodes; from += 100){
    cout<<get_avg_terminated_in_xplus(from, from+100)<<"\\% & ";
  }
  cout<< setprecision(-1);
  cout<<"\\\\ \\hline" <<endl;

  cout<<"Runs touched $X^+$ & ";
  cout<< setprecision(3);
  for(int from =0; from < statistics.num_episodes; from += 100){
    cout<<get_avg_touched_xplus(from, from+100)<<"\\%  & ";
  }
  cout<< setprecision(-1);
  cout<<"\\\\ \\hline" <<endl;

  cout<<"100\\% success contr. & ";
  cout<< setprecision(3);
  for(int from =0; from < statistics.num_episodes; from += 100){
    cout<<get_avg_terminated_all_in_xplus(from, from+100)<<"\\%  & ";
  }
  cout<< setprecision(-1);
  cout<<"\\\\ \\hline" <<endl;

  cout<<"\\"<<"end{tabular}"<<endl;
  cout<<"\\end{tiny}"<<endl<<endl;

}


void process_file(char *filename, int modus){

  statistics.num_episodes = 0;
  //  read_file("test.multi.exhaustive.1.stat");
  read_file(filename);

  cout<<"**********************************************************"<<endl;
  cout<<"Summarizing statistics for file "<<filename<<endl;
  cout<<"Found "<<statistics.num_episodes  <<" episodes"<<endl;


  print_statistics(modus);
  print_latex_statistics(modus);

}

struct seval_alltr {
  struct{
    long at_episode;
    long total_episodes;
    char filename[1000];
    double terminated_in_xplus;
    double cycles_outof_xplus;
    double cycles_in_xplus;
    double final_reward;
  } bestcontroller[1000];
  long num_trials;
} alltrials;

struct seval_sum {
  double total_episodes;
  double at_episode;
  double terminated_in_xplus;
  double cycles_outof_xplus;
  double at_episode_var;
  double terminated_in_xplus_var;
  double cycles_outof_xplus_var;
} sumstat;

void evaluate_alltrials(){
  for (int i=0;i<alltrials.num_trials;i++){
    cout<<"Trial "<<i<<" best controller_at "<<alltrials.bestcontroller[i].at_episode
	<<" of "<<alltrials.bestcontroller[i].total_episodes
	<<" Term. in X+: "<<alltrials.bestcontroller[i].terminated_in_xplus
	<<" || cycles out of X+ "<<alltrials.bestcontroller[i].cycles_outof_xplus
      //	<<" || cycles in X+ "<<alltrials.bestcontroller[i].cycles_in_xplus
      //	<<" || final reward: "<<alltrials.bestcontroller[i].final_reward
	<<" @ "<<alltrials.bestcontroller[i].filename
	<<endl;
  }


  sumstat.total_episodes=0;
  sumstat.at_episode=0;
  sumstat.terminated_in_xplus=0;
  sumstat.cycles_outof_xplus=0;
  sumstat.at_episode_var=0;
  sumstat.terminated_in_xplus_var=0;
  sumstat.cycles_outof_xplus_var=0;

  // computation of mean
  for (int i=0;i<alltrials.num_trials;i++){
    sumstat.total_episodes += alltrials.bestcontroller[i].total_episodes/ alltrials.num_trials;
    sumstat.at_episode += alltrials.bestcontroller[i].at_episode/ alltrials.num_trials;
    sumstat.terminated_in_xplus += alltrials.bestcontroller[i].terminated_in_xplus/ alltrials.num_trials;
    sumstat.cycles_outof_xplus += alltrials.bestcontroller[i].cycles_outof_xplus/ alltrials.num_trials;
  }
#define SQUARE(x) (x*x)
  // computation of standard deviation
  for (int i=0;i<alltrials.num_trials;i++){
    sumstat.at_episode_var += SQUARE((alltrials.bestcontroller[i].at_episode-sumstat.at_episode))/ (alltrials.num_trials-1);
    sumstat.terminated_in_xplus_var += SQUARE((alltrials.bestcontroller[i].terminated_in_xplus - sumstat.terminated_in_xplus))/ 
      (alltrials.num_trials-1);
    sumstat.cycles_outof_xplus_var += SQUARE((alltrials.bestcontroller[i].cycles_outof_xplus-sumstat.cycles_outof_xplus))/ (alltrials.num_trials-1);
  }

  cout<<"Average "
      <<" best controller_at "<<sumstat.at_episode<<" ("<<sqrt(sumstat.at_episode_var)<<") "
      <<" of "<<sumstat.total_episodes
      <<" Term. in X+: "<<sumstat.terminated_in_xplus<<" ("<<sqrt(sumstat.terminated_in_xplus_var)<<") "
      <<" || cycles out of X+ "<<sumstat.cycles_outof_xplus<<" ("<<sqrt(sumstat.cycles_outof_xplus_var)<<") "
      <<endl;


}

void process_all_files_in_list(char *dir_name, struct dirent **filelist, const int fcount){

#if 0
  // output all files first 
  for(int i = 0; i < fcount; i++)  {
    char filename[1000];
    sprintf(filename, "%s/%s",dir_name,filelist[i]->d_name);
    printf("Analyzing file %d: %s\n", i, filename);
    process_file(filename, 1);  
  }
#endif

  // analyze files and make overall statistic
  alltrials.num_trials= 0;
  for(int i = 0; i < fcount; i++)  {
    char filename[1000];
    sprintf(filename, "%s/%s",dir_name,filelist[i]->d_name);
    //    printf("Analyzing file %d: %s\n", i, filename);
    read_file(filename);
    int idx = get_best_controller(0,statistics.num_episodes);
    alltrials.num_trials++;
    alltrials.bestcontroller[i].at_episode 
      = idx;
    sprintf(alltrials.bestcontroller[i].filename,"%s",filename);
    alltrials.bestcontroller[i].total_episodes 
      = statistics.num_episodes;
    if(idx < 0){
      cout<<"WARNING: No best controller found in file "<< filename<<endl;
    }
    else{
      alltrials.bestcontroller[i].terminated_in_xplus 
	= statistics.data[idx][TERMINATED_IN_XPLUS];
      alltrials.bestcontroller[i].cycles_outof_xplus 
	= statistics.data[idx][CYCLES_OUTOF_XPLUS];
      alltrials.bestcontroller[i].cycles_in_xplus 
	= statistics.data[idx][CYCLES_IN_XPLUS];
      alltrials.bestcontroller[i].final_reward
	= statistics.data[idx][FINAL_REWARD];
    }
  }
  evaluate_alltrials();
}

// linux
int file_select(const struct dirent *entry)
{
  if (strstr (entry->d_name,".stat") != NULL)  // take only files that contain .stat
    return (1);
  else return 0;
}

// osx
int file_select(struct dirent *entry)
{
  if (strstr (entry->d_name,".stat") != NULL)  // take only files that contain .stat
    return (1);
  else return 0;
}

int main(int argc, char** argv) {
 struct dirent **filelist = {0};
 // char *directory = ".";
 int fcount = -1;
 int i = 0;
 
 if(argc<2){
   printf("Error: Call with %s <filename> <mode> or %s <directoryname>\n",argv[0],argv[0]);
   exit(0);
 }
 fcount = scandir(argv[1], &filelist, file_select, alphasort);

 if(fcount < 0) {
   cout<<"STATEVAL: Analyzing a single file"<<endl;
   if(argc >2)
     process_file(argv[1], atoi(argv[2]));
   else
     process_file(argv[1],0);  
   return 0;
 }

 process_all_files_in_list(argv[1],filelist, fcount);

 /*
 for(i = 0; i < fcount; i++)  {
   printf("Analyzing file %d: %s\n", i, filelist[i]->d_name);
   process_file(filelist[i]->d_name, 1);  
 }
 */

 for(i = 0; i < fcount; i++)
   free(filelist[i]);
 free(filelist);
 return 0;
}

/*
int main(int argc, char **argv){

  if(argc >1){
    if(argc >2)
      process_file(argv[1], atoi(argv[2]));
    else
      process_file(argv[1],0);
  }
  else{
    cout<<"No filename argument given. Exiting"<<endl;
    exit(0);
  }
  
}
*/
