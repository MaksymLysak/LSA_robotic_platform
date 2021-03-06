

#define MIN(a,b) (((a)<(b))?(a):(b))
#define MAX(a,b) (((a)>(b))?(a):(b))

/* Structure for encoder ticks */
typedef struct
{  uint16_t esq;
   uint16_t dir;
} Num_ticks;

/* Structure for ticks variation */
typedef struct
{  int16_t esq;
   int16_t dir;
} Var_ticks;

/* Structure for Motors PWM */
typedef struct
{ 	uint8_t esq;
	uint8_t dir;
	uint8_t en1_esq:1;
	uint8_t en2_esq:1;
	uint8_t en1_dir:1;
	uint8_t en2_dir:1;
} My_PWM;

