uint16_t derive_line[128];

int derivativeMax1_index = 0;
int derivativeMax2_index = 0;
int trigger1_index = 0;
int trigger2_index = 0;



void deriveLine(){
	int i;
	int delta;
	derive_line[0] = 0;
	for(i = 1; i < 127; i++ ){
		delta = line[i+1] - line[i];
		derive_line[i] = delta;
	}
}

void fineSpecialPoints(){
	int i;
	int current1;
	int current2;
	/*doing 2 to optimize code*/
	for(i = 0, i < 64, i ++){
		current1 = line[i];
		current2 = line[127-i];
		if(current1 > line[derivativeMax1_index){
			derivativeMax1_index = i;
		}
		else if(current1 < trigger1_index){
			trigger1_index = i;
		}
		else{
		}
		if(current2 > line[derivativeMax2_index){
			derivativeMax1_index = 127 - i;
		}
		else if(current2 < trigger2_index){
			trigger1_index = 127 - i;
		}
		else{
		}
	}
}

int main(){
	int deltaX1;
	int deltaX2;
	for(;;){
		/*gets line[] from ADC*/
		deriveLine();
		fineSpecialPoints();
		deltaX1 = 64 - derivativeMax1_index;
		deltaX2 = derivativeMax2_index - 64;
		if(deltaX1 > deltaX2){
			/*stear right*/
		}
		else if(deltaX1 < deltaX2){
			/*stear left*/
		}else{
			/*center, keep it center*/
		}
	}
	return 0;
}