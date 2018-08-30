/*
 * Logger.hpp
 *
 *  Created on: May 19, 2015
 *      Author: jerife
 */

#ifndef LOGGER_H_
#define LOGGER_H_

class Logger {
public:

	Logger(int len, int cols, double *ite);
	~Logger();

	int WriteNewItem();

	int SaveLog(const char *filename);



private:

	double *item;	// Puntero a los items (al primer elemento de una fila)
	double *log;	// Puntero a la tabla
	int idx;			// �ndice
	int len;		// Cantidad de filas
	int bw;			// Otro �ndice
	int cols;		// Cantidad de Columnas

};

#endif /* LOGGER_H_ */
