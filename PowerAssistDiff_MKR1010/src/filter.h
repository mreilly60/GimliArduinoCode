

class LowPass
{
  private:
    float a[1];
    float b[1+1];
    float omega0;
    float dt;
    bool adapt;
    float tn1 = 0;
    float x[1+1]; // Raw values
    float y[1+1]; // Filtered values

  public:  
    LowPass(float f0, float fs);
    void setCoef();
    float filt(float xn);
};