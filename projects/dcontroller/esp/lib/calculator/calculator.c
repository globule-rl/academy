#include "calculator.h"

int addition(int a, int b)
{
  return a + b;
}

int subtraction(int a, int b)
{
  return a - b;
}

int multiplication(int a, int b)
{
  return a * b;
}

int division(int a, int b)
{
  if (b == 0) {
    return 0;  // Return 0 on division by zero
  }
  return a / b;
}
