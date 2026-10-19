// So sánh bản esp-nn (fomo_espnn.c) với bản C chuẩn trên ảnh val COCO 96x96 (file .raw, 9216 B/ảnh).
// Cần mã nguồn esp-nn: git clone https://github.com/espressif/esp-nn.git && (cd esp-nn && git checkout v1.3.0)
//   E=esp-nn F=../firmware_arduino/adaremac_fomo_cam
//   gcc -O2 -DFOMO_ESPNN_HOST -I$F -I$E/include -I$E/src/common test_fomo_espnn.c $F/fomo_micro.c $F/fomo_espnn.c \
//       $E/src/convolution/esp_nn_conv_ansi.c -lm -o test_fomo_espnn
//   ./test_fomo_espnn val.raw exact.bin espnn.bin      (kết quả nhóm đo: F1 0,3074 ở cả hai bản)
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include "fomo_micro.h"
int main(int argc,char**argv){
  FILE*f=fopen(argv[1],"rb"); FILE*o1=fopen(argv[2],"wb"); FILE*o2=fopen(argv[3],"wb");
  if(fomo_fast_init()){printf("init fail\n");return 1;}
  uint8_t img[9216]; int8_t a[144],b[144]; long n=0,diff=0,maxd=0; double t1=0,t2=0;
  while(fread(img,1,9216,f)==9216){
    clock_t c=clock(); fomo_micro_run(img,a); t1+=clock()-c;
    c=clock(); fomo_micro_run_fast(img,b); t2+=clock()-c;
    for(int i=0;i<144;i++){int d=abs(a[i]-b[i]); diff+=d!=0; if(d>maxd)maxd=d;}
    fwrite(a,1,144,o1); fwrite(b,1,144,o2); n++;
  }
  printf("%ld anh: %ld/%ld logit lech (max %ld), exact %.2f ms, espnn-ansi %.2f ms\n",n,diff,n*144,maxd,t1*1000/CLOCKS_PER_SEC/n,t2*1000/CLOCKS_PER_SEC/n);
}
