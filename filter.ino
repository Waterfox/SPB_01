//#define NZEROS 3
//#define NPOLES 3
//#define GAIN   2.691197539e+01
//float xv[NZEROS+1], yv[NPOLES+1];

//float filterloop(float input_val)
//  { for (;;)
//      { xv[0] = xv[1]; xv[1] = xv[2]; xv[2] = xv[3]; 
//        xv[3] = input_val / GAIN;
//        yv[0] = yv[1]; yv[1] = yv[2]; yv[2] = yv[3]; 
//        yv[3] =   (xv[0] + xv[3]) + 3 * (xv[1] + xv[2])
//                     + (  0.1758789745 * yv[0]) + ( -0.8327215725 * yv[1])
//                     + (  1.3595771657 * yv[2]);
//        return yv[3];
//      }
//  }
