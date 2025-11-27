#include <stdio.h>
#include <string.h>

char tx_buffer[30]; // Buffer to store received data
char data_buffer[30]; // Buffer to do string operations
unsigned int x_axis_adc0 = 0;

void substr(char *dest, const char *src, unsigned int start, unsigned int count) {
	/*
		 //  char s[] = "121111110231023" ;
		 //  char t[1];
		 //  substr(t, s, 1 , 1 );
	*/
    strncpy(dest, src + start, count);
    dest[count] = 0;
}

// substring(dest, "hello", 1, 3); // prints "el"
void substring(char *dest, const char *src, unsigned int start, unsigned int end_exclusive) {
	/*
		 //  char s[] = "121111110231023" ;
		 //  char t[1];
		 //  substr(t, s, 1 , 1 );
	*/
	substr(dest, src, start, end_exclusive - start);
}


void parse_usart_incoming_stream(const char* stream, unsigned int length) {
    // char a[] = "a:1 b:2";
    memset(data_buffer,'\0', sizeof(data_buffer));

    char fields[] = { 'a', 'x' };

    int i = 0;
    int j = 0;
    while (i < length) {
    	for (int ei = 0; ei < sizeof(fields); ei++) {
    		if (stream[i] == fields[ei]) {
    			j = i + 2;
                for (;j < length; j++) {
                    if (stream[j] == ' ' || stream[j] == '\r' || stream[j] == '\n') {
                        break;
                    }
                }
                
                printf("%d %d\n", i + 2 , j);
                if (i + 2 >= (j - 1)) {
                    break;
                }

                substring(data_buffer, stream, i + 2, j);
                printf("%c: ->%s<-, ", stream[i], data_buffer);
                // convert to int and set
                switch(stream[i])
                {
                   case 'x':
                       printf("setting x...\n");
                	   sscanf(data_buffer, "%d\n",&x_axis_adc0);
                	   break;
                }
    			break;
    		}
    	}
        i++;
    }
}

int main() {
    char a[] = "a:1 x:1";
	memset(tx_buffer,'\0', sizeof(tx_buffer));

    parse_usart_incoming_stream(a, sizeof(a));
    printf("x: %d", x_axis_adc0);

    return 0;
}