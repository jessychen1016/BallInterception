const double ACC_MAX = 600;
const double VEL_MAX = 400;
const double FRAME_PERIOD = 1 / 60.0;

double* calculateVel(double start, double end) {
    double dist = abs(end - start);
    int tAcc, tFlat;	
    double vMax = sqrt(ACC_MAX * dist);
    if(vMax < VEL_MAX) {
        tAcc = (int)(vMax / ACC_MAX / FRAME_PERIOD);
        tFlat = 0;
    }
    else {
        tAcc = (int)(VEL_MAX / ACC_MAX / FRAME_PERIOD);
        double distAcc = VEL_MAX * VEL_MAX / (2 * ACC_MAX);
        tFlat = (int)((dist - distAcc * 2) / VEL_MAX / FRAME_PERIOD);
    }
    int numFrame = 2 * tAcc + tFlat;	
    double *velList = new double[numFrame];
    for (int i = 1; i <= tAcc; i++) {
        velList[i - 1] = i * ACC_MAX;
        velList[numFrame - i] = i * ACC_MAX;
    }
    for(int i = tAcc; i < tAcc + tFlat; i++) {
        velList[i] = VEL_MAX;
    }
    return velList;
}
