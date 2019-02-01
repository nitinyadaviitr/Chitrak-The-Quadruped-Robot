#include <MatrixMath.h>

#define N  (3)

mtx_type A[N][N];

mtx_type maxVal = 10;  // maxValimum random matrix entry range

void setup()
{
  Serial.begin(9600);

  // Initialize matrices
  for (int i = 0; i < N; i++)
  {          
    for (int j = 0; j < N; j++)
    {
      A[i][j] = random(maxVal) - maxVal / 2.0f; // A is random
    }
  }

}

void loop()
{
  Serial.println("\nA:");
  Matrix.Print((mtx_type*)A, N, N, "");
  Matrix.Invert((mtx_type*)A, N);
  Serial.println("\nInverted A:");
  Matrix.Print((mtx_type*)A, N, N, "A");

  while (1);
}
