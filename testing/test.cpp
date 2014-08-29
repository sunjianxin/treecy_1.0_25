#include <iostream>
#include <fstream>
#include <string>
#include <sstream>
#include <stdlib.h>
#include <vector> //std::vector
#include <typeinfo>
#include <algorithm> //std::sort
int
main (int argc, char *argv[])
{
	std::ofstream file;
	file.open("a.txt", std::fstream::app);
	file << 1 << " ";
	file.close();
}
