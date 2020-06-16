#include <thread>
#include "btcontroller.hpp"

using namespace std;

int main()
{
  remoteControl.start();
  while(remoteControl.running())
  {

  }

  return 0;
}
