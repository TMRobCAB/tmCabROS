#include "incfiles.h"
#include "Logger.h"


Logger::Logger(int len,int cols,double *item){	// Inicializa la "clase" lc
    /* Esta funci�n recibe: una clase Log, una cantidad de filas y columnas y los �tems de una fila
	   y asigna a esa clase los otros elementos que recibi�, pone los �ndices en 0 y reserva memoria*/
	this->item=item;
    idx=0;
    this->len=len;
    bw=0;
    this->cols=cols;
    log = new double[len*cols];	// Reserva memoria para la tabla log, del tama�o de len*cols
 }

Logger::~Logger(){	// Recibe una clase Log y libera la memoria de su tabla
    delete[] log;	// Libera la memoria reservada para la tabla
}

int Logger::WriteNewItem(){	// Agrega a la tabla log una fila de items en la posici�n indicada por i

	// Opera todo sobre la misma clase Log que recibe

	memcpy(&log[idx*cols],item,cols*sizeof(double));

    if(++bw >= len)	// Aumenta en 1 el �ndice bw, y si se pas� de la �ltima fila (bw==len) lo deja ah�
	bw = len;
    if(++idx >= len)	// Aumenta en 1 el �ndice i, y si se pas� de la �ltima fila (i==len) lo resetea
	idx = 0;
    return 0;
}

int Logger::SaveLog(const char *filename){	// Guarda un LogContext en un archivo

    //setlocale (LC_NUMERIC,"C");	// Establece las funciones para imprimir n�meros en "C"

    FILE *f=fopen(filename,"w");// Abre un archivo de escritura "f"

    if(f==NULL)		// Si no se abri�, hubo un error y sale
	    return -1;

    int bwLocal = this->bw;	// Copia el �ndice bw de la clase Log que recibi� (lc)

    int i = idx - bwLocal; // i es la diferencia entre los �ndices (i-bw) de lc

    if(i<0)	// Si esa diferencia es negativa (bw > i),
	i+=len;	// "Reposiciona" el nuevo �dice i entre (0<= i < len)

    int fj;

    for(fj=0;fj < cols ;fj++){	// Va recorriendo una fila de la tabla

	    fprintf(f,"%f,",log[i*cols+fj]);	// Escribe los datos de la fila i
    }

    fprintf(f,"\n");

    for(;bwLocal>0;bwLocal--){	// Escribe en el archivo una cantidad de filas (bw), desde la fila i hasta la (i - 1)

	    for(fj=0;fj < cols ;fj++){	// Va recorriendo una fila de la tabla

		    fprintf(f,"%f,",log[i*cols+fj]);	// Escribe los datos de la fila i
	    }

	    if(++i >= len)	// Aumenta i, y si se pas� de la �ltima fila, lo resetea
		i=0;

	    fprintf(f,"\n");
    }

    fclose(f);

    return 0;
}
