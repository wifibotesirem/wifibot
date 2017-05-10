#include "wifibot.h"

#define labusb

int tcpok=0;

unsigned char buffso_Raw_Data_Out[21];
unsigned char buffso_Raw_Data_In[9];

void drive(int hUSB,char speed1,char speed2);
void driveraw(int HUSB);

int main(int argc, char *argv[])
{
	buffso_send[0]=0;
	buffso_send[1]=0;
	buffso_send[2]=0;	
	buffso_send[3]=0;
	buffso_send[4]=0;
	buffso_send[5]=0;
	buffso_send[6]=0;
	debug = atoi(argv[1]);
	printf("running....%d\n",argc);
	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	//Lauch Thread
	if ((ret = pthread_create (& thread [11], NULL, Thread_RS232_33f, (void *) 10)) != 0) {
		fprintf (stderr, "%s", strerror (ret));
		exit (1);
	}
	////////////////////////////////////////////////////////////////////////////////////////////////
	////////////////////////////////////////////////////////////////////////////////////////////////
	if ((ret = pthread_create (& thread [4], NULL, Thread_Dog, (void *) 4)) != 0) {
		fprintf (stderr, "%s", strerror (ret));
		exit (1);
	}

	if ((ret = pthread_create (& thread [0], NULL, Thread_TCP, (void *) 0)) != 0) {
		fprintf (stderr, "%s", strerror (ret));
		exit (1);
	}

	if ((ret = pthread_create (& thread [1], NULL, Thread_TCP_Trooper_In, (void *) 1)) != 0) {
		fprintf (stderr, "%s", strerror (ret));
		exit (1);
	}

	if ((ret = pthread_create (& thread [2], NULL, Thread_TCP_Trooper_Out, (void *) 2)) != 0) {
		fprintf (stderr, "%s", strerror (ret));
		exit (1);
	}
	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

	//Keyboard Handler
	struct sigaction action;

	action . sa_handler = gestionnaire;
	sigemptyset (& (action . sa_mask));
	action . sa_flags = 0;

	if (sigaction (SIGQUIT, & action, NULL) != 0) {
		fprintf (stderr, "Erreur %d\n", errno);
		exit (1);
	}

	action . sa_handler = gestionnaire;
	sigemptyset (& (action . sa_mask));
	action . sa_flags = SA_RESTART | SA_RESETHAND;

	if (sigaction (SIGINT, & action, NULL) != 0) {
		fprintf (stderr, "Erreur %d\n", errno);
		exit (1);
	}
	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	while ( 1 ){
		char c=getchar();
		if (c=='p') {
			speed1+=10;
		}
		usleep(500);
	}
	return 0;
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void * Thread_TCP (void * num)
{
	int numero = (int)num;

	printf("listenTCP \n"); 

	/* Create socket for incoming connections */
	if ((socket_tcp = socket(PF_INET, SOCK_STREAM, IPPROTO_TCP)) < 0)
		printf("socket create error\n");

	/* Construct local address structure */
	memset(&myaddr_tcp, 0, sizeof(myaddr_tcp));   /* Zero out structure */
	myaddr_tcp.sin_family = AF_INET;                /* Internet address family */
	myaddr_tcp.sin_addr.s_addr = htonl(INADDR_ANY); /* Any incoming interface */
	myaddr_tcp.sin_port = htons(15020);      /* Local port */

	/* Bind to the local address */
	int autorisation=1;
	setsockopt(socket_tcp,SOL_SOCKET,SO_REUSEADDR,&autorisation,sizeof(int));

	if (bind(socket_tcp, (struct sockaddr *) &myaddr_tcp, sizeof(myaddr_tcp)) < 0)
		printf("bind error\n");

	for (;;) /* Run forever */
	{       
		printf("listenTCP \n");

		/* Mark the socket so it will listen for incoming connections */
		if (listen(socket_tcp, MAXPENDING) < 0)
			printf("listen error\n");

		/* Set the size of the in-out parameter */
		clntLen = sizeof(echoClntAddr);
		/* Wait for a client to connect */
		if ((clntSock = accept(socket_tcp, (struct sockaddr *) &echoClntAddr, &clntLen)) < 0) printf("accept error\n");
		printf("listenOK \n");

		/* clntSock is connected to a client! */
		do{
			if ((recvMsgSize = recv(clntSock,buffso_Raw_Data_In, 9, 0)) < 1) {if (debug==3) printf("recvMsgSize<1 : %d \n",recvMsgSize);shutdown(clntSock,1);}

			else{
				//pthread_mutex_lock (& mutex_dog);
				connected=1;	
				dog =0;
				//pthread_mutex_unlock (& mutex_dog);	       	          
				if (debug==0)
				{
					watchdog = 0;
					cycles = 3;
					pthread_mutex_lock (& mutex_raw_out);
					send(clntSock,buffso_Raw_Data_Out,21,0);//pb crach soft
					pthread_mutex_unlock (& mutex_raw_out);
					if (debug==3) printf("I2C passed OK\n");
				}  
				else if ((debug==1)||(debug==3))
				{
					buffso_send[0]=simu;simu++;if (simu==160) simu=0;
					send(clntSock,buffso_send,7,0);//pb crach soft		
				}
			}
		}while(recvMsgSize>0);

		if (debug==3) printf("recvMsgSize<=0 : %d \n",recvMsgSize);
		connected=0;
		shutdown(clntSock,1);
		close(clntSock);
		sleep(1);
	}//end for(;;)
}

void * Thread_TCP_Trooper_In (void * num)
{
	int numero = (int) num;
	socklen_t clilen;
	clilen=sizeof(client_in);
	socket_in=socket(PF_INET,SOCK_DGRAM,0);
	myaddr_in.sin_family=AF_INET;
	myaddr_in.sin_addr.s_addr=htonl(INADDR_ANY);
	myaddr_in.sin_port=htons(15000);
	printf("udp IN ok \n");
	bind(socket_in,(struct sockaddr *)&myaddr_in,sizeof(myaddr_in));

	while(1)
	{    
		//We wait for the client to send "init" and send "ok" in return
		recvfrom(socket_in,buffso_rcvIN,9,0,(struct sockaddr *)&client_in,&clilen);

		if(((short)((((unsigned char)buffso_rcvIN[8] << 8) + (unsigned char)buffso_rcvIN[7]))) == crc16((unsigned char*)(buffso_rcvIN+1),6))
		{
			watchdog = 0;
			//pthread_mutex_lock (& mutex_send);
			memcpy(buffso_Raw_Data_In,buffso_rcvIN,9);
			//pthread_mutex_unlock (& mutex_send);

#ifdef debug_msg 
			printf("speed1 %d speed2 %d \n",speed1,speed2);
#endif
			cycles = 3;
			servermode = 0;
		}//end of else if
	}//end of main loop
}

void * Thread_TCP_Trooper_Out (void * num)
{
	int numero = (int) num;
	int clilen;
	char ping_buffer[30];
	char ac[80];
	struct hostent *phe;
	struct sockaddr_in addr;
	clilen=sizeof(client_out);
	socket_out=socket(PF_INET,SOCK_DGRAM,0);
	myaddr_out.sin_family=AF_INET;
	myaddr_out.sin_addr.s_addr=htonl(INADDR_ANY);
	myaddr_out.sin_port=htons(15010);
	printf("udp OUT ok \n");
	bind(socket_out,(struct sockaddr *)&myaddr_out,sizeof(myaddr_out));

	while(1)
	{	memset(buffso_Raw_Data_Out,0,21);
		//We wait for the client to send "init" and send "ok" in return
		recvfrom(socket_out,buffso_rcvOUT,21,0,(struct sockaddr *)&client_out,&clilen);

		if(!(strstr(buffso_rcvOUT,"init")==NULL))
		{
			printf("udp OUT init ok \n");	
			sprintf(buffso_sendOUT,"ok");
			sendto(socket_out,buffso_sendOUT,3,0,(struct sockaddr *)&client_out,sizeof(struct sockaddr_in));
		}
		else if(!(strstr(buffso_rcvOUT,"data")==NULL))
		{  
			//pthread_mutex_lock (& mutex_raw_out);
			short mycrcsend = crc16((unsigned char*)buffso_Raw_Data_Out,18);
			buffso_Raw_Data_Out[19]=(unsigned char)mycrcsend;
			buffso_Raw_Data_Out[20]=(unsigned char)(mycrcsend >> 8);
			sendto(socket_out,buffso_Raw_Data_Out,21,0,(struct sockaddr *)&client_out,sizeof(struct sockaddr_in));
			//pthread_mutex_lock (& mutex_raw_out);
		}//end of else if
	}//end of main loop
}

void * Thread_RS232_33f (void * num)
{
	int numero = (int) num;
	unsigned char sbuf[5];
	
	hUSB = openrs232("/dev/ttyS0",19200);

	close(hUSB);
	sleep(8);
	
	hUSB = openrs232("/dev/ttyS0",19200);

	speed1=0;speed2=0;	
	SetRS232Motor33f_low_res(hUSB,0x00,0x00);
	SetRS232MotorPID33f(hUSB,0x00,0x00,80,45,0,360);
	
	while (1) {

		//pthread_mutex_lock (& mutex_send);
		driveraw(hUSB);		
		//pthread_mutex_unlock (& mutex_send);
		GetRS232Motor33f(hUSB,&dataL,&dataR);
		//printf("odors232R %ld L%ld bat%ld Ia%ld speed L %ld speedR %ld\n",dataR.odometry,dataL.odometry,dataL.robot_voltage,dataL.robot_current,dataL.SpeedFront,dataR.SpeedFront);
	}
}

int SetRS232Motor33f_low_res(int hUSB,char speed1,char speed2) {

	unsigned int n;
	unsigned char sbuf[20];
	int ress=0;
	unsigned char tt=0;
	sbuf[0] = 255;
	sbuf[1] = 0x07;			
	int tmp1 = 8*(speed1&0x3F);
	int tmp2 = 8*(speed2&0x3F);
	if (speed2&0x80) tt=tt+32;
	if (speed2&0x40) tt=tt+16;
	sbuf[2] = (unsigned char)tmp1;
	sbuf[3] = (unsigned char)(tmp1 >> 8);
	sbuf[4] = (unsigned char)tmp2;
	sbuf[5] = (unsigned char)(tmp2 >> 8);

	sbuf[6] = (unsigned char)((speed1&0x80) + (speed1&0x40) + tt);

	short mycrcsend = crc16(sbuf+1,6);

	sbuf[7] = (unsigned char)mycrcsend;
	sbuf[8] = (unsigned char)(mycrcsend >> 8);

	ress = writers232(hUSB, sbuf, 9,&n);
	return ress;
}

void driveraw(int hUSB)
{
	unsigned int n;
	unsigned char sbuf[20];
	int ress=0;

	char address=128;
	char mflag1=0;
	char mflag2=0;

	ress = writers232(hUSB, buffso_Raw_Data_In, 9,&n);
}


SetRS232Motor33f(int hUSB,short speed1,short speed2,unsigned char SpeedFlag) {

	unsigned int n;
	unsigned char sbuf[20];
	int ress=0;
	unsigned char tt=0;
	sbuf[0] = 255;
	sbuf[1] = 0x07;			
	sbuf[2] = (unsigned char)speed1;
	sbuf[3] = (unsigned char)(speed1 >> 8);
	sbuf[4] = (unsigned char)speed2;
	sbuf[5] = (unsigned char)(speed2 >> 8);

	sbuf[6] = SpeedFlag;

	short mycrcsend = crc16(sbuf+1,6);

	sbuf[7] = (unsigned char)mycrcsend;
	sbuf[8] = (unsigned char)(mycrcsend >> 8);

	ress = writers232(hUSB, sbuf, 9,&n);
	return ress;
}

int SetRS232MotorPID33f(int hUSB,char speed1,char speed2,char pp,char ii,char dd,short maxspeed) {

	unsigned int n;

	char sbuf[20];
	int ress=0;

	sbuf[0] = 255;
	sbuf[1] = 0x09;
	sbuf[2] = speed1;
	sbuf[3] = speed2;
	sbuf[4] = pp;
	sbuf[5] = ii;
	sbuf[6] = dd;
	sbuf[7] = (char)maxspeed;
	sbuf[8] = (char)(maxspeed >> 8);

	short mycrcsend = crc16(sbuf+1,8);
	sbuf[9] = (unsigned char)mycrcsend;
	sbuf[10] = (unsigned char)(mycrcsend >> 8);

	ress = writers232(hUSB, sbuf, 11, &n);
	return ress;
}

int StopMotorRS23233f(int hUSB){
	SetRS232Motor33f_low_res(hUSB,0x00,0x00);
	return 1;
}

int GetRS232Motor33f(int hUSB,SensorData *dataL,SensorData *dataR) {

	int n;
	unsigned char sbuf[30];
	int ress=0;
	int r=0;

	do { // Sync
		r = readrs232(hUSB,sbuf, 1, &n);
	} while (sbuf[0] != 255 && r);

	if (r) r = readrs232(hUSB,sbuf, 21, &n);

	short mycrcrcv = (short)((sbuf[20] << 8) + sbuf[19]);
	short mycrcsend = crc16(sbuf,19);

	if (mycrcrcv!=mycrcsend)
	{
		do { // Sync
			r = readrs232(hUSB,sbuf, 1, &n);
		} while (sbuf[0] != 255 && r);
	}
	else 
	{
		dataL->SpeedFront=(int)((sbuf[1] << 8) + sbuf[0]);
		if (dataL->SpeedFront > 32767) dataL->SpeedFront=dataL->SpeedFront-65536;
		dataL->SpeedFront=dataL->SpeedFront;
		dataL->robot_voltage=sbuf[2];
		dataL->IR=sbuf[3];
		dataL->IR2=sbuf[4];
		dataL->odometry=((((long)sbuf[8] << 24))+(((long)sbuf[7] << 16))+(((long)sbuf[6] << 8))+((long)sbuf[5]));

		dataR->SpeedFront=(int)((sbuf[10] << 8) + sbuf[9]);
		if (dataR->SpeedFront > 32767) dataR->SpeedFront=dataR->SpeedFront-65536;
		dataR->SpeedFront=dataR->SpeedFront;
		dataR->robot_voltage=sbuf[1];
		dataR->IR=sbuf[11];
		dataR->IR2=sbuf[12];
		dataR->odometry=((((long)sbuf[16] << 24))+(((long)sbuf[15] << 16))+(((long)sbuf[14] << 8))+((long)sbuf[13]));
		dataL->robot_current=sbuf[17];
		dataR->robot_current=sbuf[17];
		dataL->robot_version=sbuf[18];
		dataR->robot_version=sbuf[18];
		//pthread_mutex_lock (& mutex_raw_out);
		memcpy(buffso_Raw_Data_Out,sbuf,21);
		//pthread_mutex_unlock (& mutex_raw_out);
	}
	return ress;
}

void * Thread_Dog (void * num)
{
	int numero = (int) num;
	while (1) {
		if(watchdog < cycles) watchdog = watchdog + 1;
		else if(watchdog == cycles)
		{		
#ifdef labusb 
			speed1=0;
			speed2=0;
#endif
			buffso_Raw_Data_In[2] = (char)speed1;
			buffso_Raw_Data_In[3] = (char)(speed1 >> 8);
			buffso_Raw_Data_In[4] = (char)speed2;
			buffso_Raw_Data_In[5] = (char)(speed2 >> 8);

			if (connected) {
				connected=0;
				int ret=pthread_cancel (thread[0]);
				shutdown(clntSock,1);       	
				close(clntSock);
				close(servSock);
				close(socket_tcp);
				sleep(4);
				if ((ret = pthread_create (& thread [0], NULL, Thread_TCP, (void *) 1)) != 0) {
					fprintf (stderr, "%s", strerror (ret));
					exit (1);
				}
			}
		}//end of else if watchdog
		usleep(500000);
	}
}

void gestionnaire (int numero)
{
	switch (numero) {
		case SIGQUIT :
			//fprintf (stdout, "\nSIGQUIT reçu\n"); fflush (stdout);		        	
			break;
		case SIGINT :
			//fprintf (stdout, "\nSIGINT reçu\n"); fflush (stdout);
			StopMotorRS23233f(hUSB);			
			close(hUSB);			
			break;
	}
}

int openrs232(const char *device, int baudrate)
{
	struct termios  options;
	int bitrate;

	//if (hUSB)
	//return 0;

	bitrate = (baudrate == 9600) ? B9600 : (baudrate == 19200) ? B19200 : 0;
	if (!bitrate)
	{
		printf("ERROR : Serial %d baudrate unsupported!\n", baudrate);
		return 0;
	}

	hUSB = open(device, O_RDWR | O_NOCTTY);

	if (hUSB < 0)
	{
		hUSB = 0;
		printf("ERROR : Serial\n");
		return 0;
	}

	tcgetattr(hUSB, &options);

	options.c_cflag = bitrate | CS8 | CREAD | CLOCAL;
	options.c_iflag = IGNPAR;
	options.c_oflag = 0;
	options.c_lflag = 0;

	options.c_cc[VMIN]  = 1;
	options.c_cc[VTIME] = 0;

	tcsetattr(hUSB, TCSANOW, &options);

	return hUSB;
}
int readrs232(int hUSB, unsigned char *buffer, unsigned int nNumberOfBytesToRead, unsigned int *lpNumberOfBytesRead)
{
	int kk=0;
	if (!hUSB)
		return -1;

	*lpNumberOfBytesRead  = 0;
	for (kk = 0; kk < nNumberOfBytesToRead; kk++)
		*lpNumberOfBytesRead += read(hUSB, buffer + kk, 1);

	//  *lpNumberOfBytesRead = ::read(m_handle, buffer, nNumberOfBytesToRead);
	return (nNumberOfBytesToRead >= 0);
}


int closers232()
{
	if (!hUSB)
		return 0;

	close(hUSB);
	hUSB = 0;

	return 1;
}

int writers232(int hUSB, unsigned char *buffer, unsigned int nNumberOfBytesToWrite, unsigned int *lpNumberOfBytesWritten)
{
	if (!hUSB)
		return 0;

	*lpNumberOfBytesWritten = 0;
	int kk=0;  
	for (kk = 0; kk < nNumberOfBytesToWrite; kk++)
		*lpNumberOfBytesWritten += write(hUSB, buffer + kk, 1);

	//*lpNumberOfBytesWritten = ::write(m_handle, buffer, nNumberOfBytesToWrite);
	return (*lpNumberOfBytesWritten >= 0);
}

short crc16(unsigned char *adresse_tab , unsigned char taille_max)
{
	unsigned int Crc = 0xFFFF;
	unsigned int Polynome = 0xA001;
	unsigned int CptOctet = 0;
	unsigned int CptBit = 0;
	unsigned int Parity= 0;

	Crc = 0xFFFF;
	Polynome = 0xA001; // Polynôme = 2^15 + 2^13 + 2^0 = 0xA001.

	for ( CptOctet= 0 ; CptOctet < taille_max ; CptOctet++)
	{
		Crc ^= *( adresse_tab + CptOctet); //Ou exculsif entre octet message et CRC

		for ( CptBit = 0; CptBit <= 7 ; CptBit++) /* Mise a 0 du compteur nombre de bits */
		{
			Parity= Crc;
			Crc >>= 1; // Décalage a droite du crc
			if (Parity%2 == 1) Crc ^= Polynome; // Test si nombre impair -> Apres decalage à droite il y aura une retenue
		} // "ou exclusif" entre le CRC et le polynome generateur.
	}
	return(Crc);
}
