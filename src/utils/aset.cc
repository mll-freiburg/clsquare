#include "aset.h"
#include "str2val.h"

Aset::Aset()
{

}

bool Aset::parseStr(const char * str_to_parse, int u_dim)
{
  const char *dum = str_to_parse;
  bool combineActions=false;

  action.clear();

  if (strskip(dum,"{",dum) != 0) combineActions=true;
  
  if (combineActions)
    {
      std::vector<Aset_action_t> actions_to_combine;
      
      for (int i=0; i<u_dim; i++) 
	{
	  Aset_action_t akt_dim_actions;
	  bool toBeRead=true;
	  do {
	    float val;
	    if (str2val(dum,val,dum) !=0)
	      akt_dim_actions.push_back(val);
	    else 
	      toBeRead=false;
	  } while (toBeRead);
	  
	  if (akt_dim_actions.size() == 0)
	    {
	      std::cerr << __PRETTY_FUNCTION__ << "WARNUNG: Actionset leer!\n";
	      return false;
	    }
	  
	  actions_to_combine.push_back(akt_dim_actions);
	  
	  strskip(dum,"}",dum);

	  if (i!=u_dim-1) strskip(dum,"{",dum);
	}
      
      //berechne Anzahl der resultierenden Aktionen:
      int n=1;
      for (int i=0; i<(int)actions_to_combine.size(); i++) n*=actions_to_combine[i].size();
      
      for (int i=0; i<n; i++)
	{
	  Aset_action_t a;
	  action.push_back(a);
	}
      
      // Betrachte Anzahl der Dimensionen nacheinander
      for (int i=0; i<(int)actions_to_combine.size(); i++){
	//Berechne Anzahl
	int m=1; for (int h=i+1; h<(int)actions_to_combine.size(); h++) m*=actions_to_combine[h].size();	
	
	for (int k=0; k < (int) actions_to_combine[i].size(); k++)
	  {
	    for(int g=0; g < (int) (n/(m* actions_to_combine[i].size())); g++) {
	      for (int h=0; h<m; h++)
		{
		  action[(g*m*actions_to_combine[i].size())+(k*m) + h].push_back(actions_to_combine[i][k]);
		}
	    }
	  }
	
      }
	return true;
    }
  
  // Direkte Eingabe der Aktionen in Form u1 u2 ... un; u1_2 u2_2 ... un2; ... 
  bool res=true;
  do {
    Aset_action_t akt_action;
    for (int i=0; i< u_dim; i++)
      {
	float val;
	if (str2val(dum,val,dum) !=0)
	  akt_action.push_back(val);
      }
    strskip(dum,";",dum);
    if ((int) akt_action.size() == u_dim)
      action.push_back(akt_action);
    else res = false;
  } while (res);

  if (action.size() == 0) std::cerr << __PRETTY_FUNCTION__ << "WARNUNG: Actionset leer!\n";

  return true;
}


void Aset::print(std::ostream & out)
{
  out << "--- begin actionset: ---------\n";
  out << " Anzahl der Aktionen: " << action.size() << "\n";
  for (int i=0; i<(int) action.size(); i++){
    out << "Action[" << i << "]  ";
    for (int h=0; h<(int)action[i].size(); h++)
      {
	out << action[i][h] << "  ";
      }
    out << "\n";
  }
  
  out << "--- end actionset. ---------\n";
}


bool Aset::getAction(int idx, float * u, int u_dim)
{
  if (idx < 0 || idx> (int)action.size()) 
    {
      std::cerr << __PRETTY_FUNCTION__ << "WARNUNG: Akionsindex " << idx << " nicht vorhanden\n";
      return false;
    }
  if ((int)action[idx].size()!=u_dim)
    {
      std::cerr << __PRETTY_FUNCTION__ << "WARNUNG: u_dim (" << u_dim << ")!= action dim (" << (int)action[idx].size() << ") der angeforderten Aktion\n";
      return false;
    }

  for (int i=0; i< u_dim; i++)
    u[i]=action[idx][i];

  return true;
}

bool Aset::getAction(int idx, double * u, int u_dim)
{
  if (idx < 0 || idx> (int)action.size()) 
    {
      std::cerr << __PRETTY_FUNCTION__ << "WARNUNG: Akionsindex " << idx << " nicht vorhanden\n";
      return false;
    }
  if ((int)action[idx].size()!=u_dim)
    {
      std::cerr << __PRETTY_FUNCTION__ << "WARNUNG: u_dim (" << u_dim << ")!= action dim (" << (int)action[idx].size() << ") der angeforderten Aktion\n";
      return false;
    }

  for (int i=0; i< u_dim; i++)
    u[i]=action[idx][i];

  return true;
}
