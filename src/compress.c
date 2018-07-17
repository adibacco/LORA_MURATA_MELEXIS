
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include "compress.h"


static RANGE range_info[] = { { 0, 0, 0 }, { 3, -4, 3}, { 4, -8, 7 }, { 8, -128, 127 } };

static QUEUE queue;


int push(int nbits, uint32_t val)
{
	int res = -1;
	if (nbits <= (32 - queue.num)) {
		queue.store = queue.store << nbits;
		queue.store |= val & ((1 << nbits) - 1);
		queue.num += nbits;
		//printf("-> %d %02x rem %d\n",  nbits, val & 0xff, queue.num);

		res = nbits;
	}

	return res;
}

int remaining() {
	return queue.num;
}


int empty() {
	int rem = queue.num;
	queue.num = 0;
	queue.store = 0;

	return rem;
}


int pop(int nbits, uint32_t* val)
{
	int res = -1;

	if (nbits <= queue.num) {
		uint32_t mask = ((1 << nbits) - 1);
		uint32_t aux = (queue.store >> (queue.num - nbits)) &  mask;
		queue.num -= nbits;
		*val = aux;
		res = nbits;
		//printf("<- %d %02x rem %d\n", nbits, *val & 0xff, queue.num);
	}

	if (queue.num == 0)
		queue.store = 0;
	return res;
}


int compress(int ilen, uint8_t* ia,  uint8_t* oa, int lim)
{

  int8_t* tmp = (int8_t*) ia;
  int8_t aux0, aux1 = 0;

  uint32_t val;

  aux0 = aux1 = tmp[0] = ia[0];
  for (int i = 1; i < ilen; i++) {
	aux0 = aux1;
	aux1 = ia[i];

    tmp[i] = aux1 - aux0;
  }

  push(8, tmp[0]);
  int j = 0;
  int nb = 0;

  for (int i = 1; i < ilen; i++)
  {
	    if (tmp[i] == 0) {
	    	push(2, 0b00);
	    } else if ((tmp[i] > range_info[0b01].min) && (tmp[i] < range_info[0b01].max)) {
	    	push(2, 0b01);
	    	push(range_info[0b01].field_len, tmp[i]);
	    } else if ((tmp[i] > range_info[0b10].min) && (tmp[i] < range_info[0b10].max)) {
	    	push(2, 0b10);
	    	push(range_info[0b10].field_len, tmp[i]);
	    } else  {
	    	push(2, 0b11);
	    	push(range_info[0b11].field_len, tmp[i]);
	    }


	    while (pop(8, &val) != -1)
	    {
	    	oa[j++] = val & 0xff;
	    	if (j >= lim)
	    		break;
	    }


  }

  int rem = remaining();

  pop(rem, &val);

  oa[j++] = ((val << (8-rem)) | ((1 << (8-rem)) - 1));

  return (j);
}

int decompress(int len, uint8_t* ia, uint8_t* oa)
{
	uint32_t val;
	int8_t* tmp = (int8_t*) oa;

	oa[0] = ia[0];
	int j = 1;
	int i = 1;
	do {
		while ((i < len) && (push(8, ia[i]) != -1))
			i = i + 1;

		pop(2, &val);
		if (val == 0b00) {
			tmp[j] = tmp[j-1];
		} else if (val == 0b01) {
			pop(range_info[0b01].field_len, &val);
			if (val & (1 << (range_info[0b01].field_len - 1))) // sign extend
				val |= (0xffffffff << range_info[0b01].field_len);
			tmp[j] =  tmp[j-1] + (int8_t) val;
		} else if (val == 0b10) {
			pop(range_info[0b10].field_len, &val);
			if (val & (1 << (range_info[0b10].field_len-1))) // sign extend
				val |= (0xffffffff << range_info[0b10].field_len);
			tmp[j] = tmp[j-1] +  (int8_t) val;
		} else  {
			// it can be a terminator
			int rem = remaining();
			if (rem >= 8) {
				pop(range_info[0b11].field_len, &val);
				if (val & (1 << (range_info[0b11].field_len - 1))) // sign extend
					val |= (0xffffffff << range_info[0b11].field_len);
				tmp[j] =  tmp[j-1] + (int8_t) val;
			}
			else {
				empty();
			}
		}
		j += 1;
	} while (remaining() > 0);

	return j - 1;
}

