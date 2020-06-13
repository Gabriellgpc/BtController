#include <iostream>
#include <iomanip>      // std::setw

using namespace std;

int main()
{
    cout << "Digite aí:";
    char key;

    cout << setw(10);
    while(true)
    {
      // cin.ignore(1, '\n');
      key = cin.get();
      if(key != '\n')
      {
        system("clear");
        cout << "Entrada: " << key;
        cout << "\nDEC:" << dec << int(key);
        cout << "\nHEX:0x" << hex << int(key);
        cout << "\nDigite aí:";
      }
    }

    return 0;
}
