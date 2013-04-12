#define main tutorial1
#include "evds_tutorial1.c"
#undef main
#define main tutorial2
#include "evds_tutorial2.c"
#undef main
#define main tutorial3
#include "evds_tutorial3.c"
#undef main
#define main tutorial5
#include "evds_tutorial5.c"
#undef main

void main() {
  tutorial3();
}
