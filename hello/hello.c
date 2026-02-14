#include <stdio.h>

int square(int x) {
  return x * x;
}

int main() {
  int result = square(5);
  printf("Result: %d\n", result);
  return 0;
}
