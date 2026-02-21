#include <iostream>
#include <cstdlib>



int main(){
	srand(time(0))
	float p = 0.2;
	float q = 0.3;



	float random = (1.0 * rand()) / (1.0 *RAND_MAX);
	countFirst = 0;

	while(1){
		random = (1.0 * rand()) / (1.0 *RAND_MAX);
		if(random < p){
			countFirst++;
			break;
		}
		countFirst++;


	}


	return 0;
}