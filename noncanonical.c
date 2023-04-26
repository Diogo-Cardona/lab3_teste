/*Non-Canonical Input Processing*/

#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <termios.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#define BAUDRATE B38400
#define _POSIX_SOURCE 1 /* POSIX compliant source */
#define FALSE 0
#define TRUE 1

volatile int STOP=FALSE;

//////////////////////////////////////////////////////////////
//MAQUINA DE ESTADOS
typedef enum{
    Start,
    FLAG_RCV,
    A_RCV,
    C_RCV,
    BCC_OK,
    MSTOP
}Estados;
 
//Variável para a máquina de estados 
Estados estado = Start;
 
void state_machine(unsigned char msg)
{
    unsigned char codigo[] = {0x5c, 0x01, 0x03, (0x01)^(0x03), 0x5c};
    switch(estado)
    {
    case Start:
           printf("start\n");
        if (msg==codigo[0])
            estado = FLAG_RCV;
        break;
    case FLAG_RCV:
                   printf("flag_rcv\n");
        if (msg==codigo[0])
            break;
        if (msg==codigo[1])
            estado = A_RCV;
        else
            estado = Start;
        break;
    case A_RCV:
        printf("a_rcv\n");
        if(msg==codigo[0])
            estado = FLAG_RCV;
        break;
        if(msg==codigo[3])
            estado = C_RCV;
        else
            estado = Start;
        break;
    case C_RCV:
        printf("c_rcv\n");
        if(msg==codigo[0])
            estado = FLAG_RCV;
        if(msg==codigo[4])
            estado = BCC_OK;
        else
            estado = Start;
    case BCC_OK:
        printf("BCC_OK\n");
        if(msg==codigo[5])
            estado=MSTOP;
        else
            estado = Start;
    default:
        break;
    }
}
//////////////////////////////////////////////////////
int main(int argc, char** argv)
{
    int fd,c, res;
    struct termios oldtio,newtio;
    char buf[255];

    if ( (argc < 2) ||
         ((strcmp("/dev/ttyS0", argv[1])!=0) &&
          (strcmp("/dev/ttyS1", argv[1])!=0) )) {
        printf("Usage:\tnserial SerialPort\n\tex: nserial /dev/ttyS1\n");
        exit(1);
    }


    /*
    Open serial port device for reading and writing and not as controlling tty
    because we don't want to get killed if linenoise sends CTRL-C.
    */


    fd = open(argv[1], O_RDWR | O_NOCTTY );
    if (fd < 0) { perror(argv[1]); exit(-1); }

    if (tcgetattr(fd,&oldtio) == -1) { /* save current port settings */
        perror("tcgetattr");
        exit(-1);
    }

    bzero(&newtio, sizeof(newtio));
    newtio.c_cflag = BAUDRATE | CS8 | CLOCAL | CREAD;
    newtio.c_iflag = IGNPAR;
    newtio.c_oflag = 0;

    /* set input mode (non-canonical, no echo,...) */
    newtio.c_lflag = 0;

    newtio.c_cc[VTIME]    = 0;   /* inter-character timer unused */
    newtio.c_cc[VMIN]     = 1;//previous 5   /* blocking read until 5 chars received */

    /*
    VTIME e VMIN devem ser alterados de forma a proteger com um temporizador a
    leitura do(s) próximo(s) caracter(es)
    */


    tcflush(fd, TCIOFLUSH);
    sleep(1);
    if (tcsetattr(fd,TCSANOW,&newtio) == -1) {
        perror("tcsetattr");
        exit(-1);
    }

    printf("New termios structure set\n");
        
    char response[strlen(buf)+1];
    int  i=0;
    unsigned char msg;
    while (STOP==FALSE) {       /* loop for input */
        res = read(fd,buf,255);  /* returns after 5 chars have been input */     
        buf[res]=0;               /* so we can printf... */     
        printf(":%s:%d\n", buf, res);
        msg = buf;
        printf("vcr=0x%02x\n", (unsigned int)buf);
        state_machine(buf);
        
        if (estado==MSTOP) STOP=TRUE;
    }


    
    

    /*
    O ciclo WHILE deve ser alterado de modo a respeitar o indicado no guião
    */
    sleep(1);
    tcsetattr(fd,TCSANOW,&oldtio);
    close(fd);
    return 0;
}
