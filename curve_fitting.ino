// Pin assignment
#define PIN_IR A0

// Configurable parameters
#define _NPOINTS   7      // 0, 5, 10, ... , 30 cm
#define _STEP_MM   50     // distance step in mm
#define _NSAMPLES  21     // number of measurements for median (odd number)
#define _MAX_DEG   5      // maximum degree of polynomial
#define _SCALE     1023.0 // t = sensor reading / _SCALE

// Global variables
float xs[_NPOINTS];       // sensor readings (0 ~ 1023)
float ys[_NPOINTS];       // actual distances (mm)
int method = 1;           // 1: polynomial, 2: inverse (y = c0 + c1 / t)
int degree = 2;           // degree of polynomial
int nparam = 3;           // number of coefficients
float coef[_MAX_DEG + 1]; // fitted coefficients

// Read a line from the serial monitor (returns when Enter is pressed)
int read_line(char *buf, int size)
{
  int len = 0;
  while (1) {
    if (Serial.available() > 0) {
      char c = Serial.read();
      if (c == '\n' || c == '\r') {
        delay(30);
        while (Serial.available() > 0)
          Serial.read();
        buf[len] = 0;
        return len;
      }
      if (len < size - 1)
        buf[len++] = c;
    }
  }
}

int read_int()
{
  char buf[16];
  read_line(buf, sizeof(buf));
  return atoi(buf);
}

// Take a median value from multiple measurements
float ir_sensor_median()
{
  int v[_NSAMPLES];

  for (int i = 0; i < _NSAMPLES; i++) {
    v[i] = analogRead(PIN_IR);
    delay(10);
  }
  // insertion sort
  for (int i = 1; i < _NSAMPLES; i++) {
    int key = v[i];
    int j = i - 1;
    while (j >= 0 && v[j] > key) {
      v[j + 1] = v[j];
      j--;
    }
    v[j + 1] = key;
  }
  return v[_NSAMPLES / 2];
}

// Basis functions (t = x / _SCALE)
//   method 1: y = c0 + c1 * t + c2 * t^2 + ...
//   method 2: y = c0 + c1 / t
float basis(int k, float x)
{
  if (x < 1) x = 1;
  float t = x / _SCALE;

  if (method == 1) {
    float r = 1.0;
    for (int i = 0; i < k; i++)
      r *= t;
    return r;
  }
  return (k == 0) ? 1.0 : 1.0 / t;
}

float fitted(float x)
{
  float y = 0;
  for (int k = 0; k < nparam; k++)
    y += coef[k] * basis(k, x);
  return y;
}

// Gaussian elimination with partial pivoting
// A is an augmented matrix (n x (n + 1))
bool solve(float A[][_MAX_DEG + 2], int n)
{
  for (int i = 0; i < n; i++) {
    int p = i;
    for (int r = i + 1; r < n; r++)
      if (fabs(A[r][i]) > fabs(A[p][i])) p = r;
    if (fabs(A[p][i]) < 1e-12)
      return false;
    for (int c = 0; c <= n; c++) {
      float tmp = A[i][c];
      A[i][c] = A[p][c];
      A[p][c] = tmp;
    }
    for (int r = i + 1; r < n; r++) {
      float f = A[r][i] / A[i][i];
      for (int c = i; c <= n; c++)
        A[r][c] -= f * A[i][c];
    }
  }
  for (int i = n - 1; i >= 0; i--) {
    float s = A[i][n];
    for (int c = i + 1; c < n; c++)
      s -= A[i][c] * coef[c];
    coef[i] = s / A[i][i];
  }
  return true;
}

// Least squares fitting (normal equations)
bool curve_fit()
{
  float A[_MAX_DEG + 1][_MAX_DEG + 2];

  for (int i = 0; i < nparam; i++) {
    for (int j = 0; j < nparam; j++) {
      float s = 0;
      for (int p = 0; p < _NPOINTS; p++)
        s += basis(i, xs[p]) * basis(j, xs[p]);
      A[i][j] = s;
    }
    float s = 0;
    for (int p = 0; p < _NPOINTS; p++)
      s += basis(i, xs[p]) * ys[p];
    A[i][nparam] = s;
  }
  return solve(A, nparam);
}

float r_squared()
{
  float mean = 0;
  for (int p = 0; p < _NPOINTS; p++)
    mean += ys[p];
  mean /= _NPOINTS;

  float ss_res = 0, ss_tot = 0;
  for (int p = 0; p < _NPOINTS; p++) {
    float e = ys[p] - fitted(xs[p]);
    ss_res += e * e;
    ss_tot += (ys[p] - mean) * (ys[p] - mean);
  }
  return 1.0 - ss_res / ss_tot;
}

void print_term(float c, bool first)
{
  if (first) {
    Serial.print(c, 6);
  } else {
    Serial.print(c < 0 ? " - " : " + ");
    Serial.print(fabs(c), 6);
  }
}

// Print the equation as an Arduino function
void print_equation()
{
  Serial.println("float volt_to_distance(unsigned int a_value)");
  Serial.println("{");
  Serial.println("  if (a_value < 1) a_value = 1;");
  Serial.println("  float t = a_value / 1023.0;");
  Serial.print("  return ");
  for (int k = 0; k < nparam; k++) {
    print_term(coef[k], k == 0);
    if (k == 0) continue;
    if (method == 1) {
      if (k == 1) {
        Serial.print(" * t");
      } else {
        Serial.print(" * pow(t, ");
        Serial.print(k);
        Serial.print(")");
      }
    } else {
      Serial.print(" / t");
    }
  }
  Serial.println(";");
  Serial.println("}");
}

void setup()
{
  Serial.begin(115200);
  while (!Serial)
    ;
}

void loop()
{
  char buf[4];

  // Get fitting options from the serial monitor
  do {
    Serial.print("method (1: polynomial, 2: inverse): ");
    method = read_int();
    Serial.println(method);
  } while (method != 1 && method != 2);

  if (method == 1) {
    do {
      Serial.print("degree (1 ~ ");
      Serial.print(_MAX_DEG);
      Serial.print("): ");
      degree = read_int();
      Serial.println(degree);
    } while (degree < 1 || degree > _MAX_DEG);
    nparam = degree + 1;
  } else {
    nparam = 2;
  }

  // Collect sensor readings (press Enter at each position)
  for (int i = 0; i < _NPOINTS; i++) {
    ys[i] = (float)i * _STEP_MM;
    Serial.print("distance: ");
    Serial.print(ys[i] / 10.0, 0);
    Serial.print(" cm, press Enter ");
    read_line(buf, sizeof(buf));
    xs[i] = ir_sensor_median();
    Serial.print("reading: ");
    Serial.println(xs[i], 0);
  }

  // Curve fitting
  if (!curve_fit()) {
    Serial.println("fitting failed");
    while (1) ;
  }
  Serial.print("R2: ");
  Serial.println(r_squared(), 6);
  print_equation();

  while (1) ;
}
