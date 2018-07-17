#ifndef _COMPRESS_H_
#define _COMPRESS_H_

typedef struct _RANGE {
	int field_len;
	int min;
	int max;
} RANGE;

typedef struct _QUEUE {
	int num;
	uint32_t store;
} QUEUE;


int compress(int ilen, uint8_t* ia,  uint8_t* oa, int lim);

#endif

