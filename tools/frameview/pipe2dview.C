#include <stdio.h>
#include <iostream>
#include <iomanip>
#include <stdlib.h>
#include <string.h>
#include <iostream>


/* funktioniert nicht richtig */
int main(int argc, char ** argv) {
  FILE *out;
  const int BUFFER_MAX_SIZE= 256;
  char buf[BUFFER_MAX_SIZE];

  out = popen("2dview -ascii","w");
  while (true) {
    std::cout << "\n>";
    std::cin.getline(buf,BUFFER_MAX_SIZE);
    if (strcmp(buf,"q")==0)
      break;
    fputs(buf,out);
    fflush(out);
  }
  fflush(out);
  pclose(out); 
  return 0;
}
