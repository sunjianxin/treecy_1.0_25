#include <iostream>
#include <stdlib.h>

int
main (int argc, char *argv[])
{
	int num;
	num = atoi(argv[1]);
	std::cout << "HappyDog! plays " << num << " times dota"<< std::endl;
}
