

// Inverts a 3x3 matrix https://ardoris.wordpress.com/2008/07/18/general-formula-for-the-inverse-of-a-3x3-matrix/
std::vector<std::vector<float>> matrixInverse_3x3( std::vector<std::vector<float>> &m) {
     // Set Up Matrix Variable    
    std::vector<std::vector<float>> o;
    for (int i=0; i < 3; i++) {
        o.push_back(std::vector<float>());
        for (int j=0; j < 3; j++) {
            o[i].push_back(0);
        }
    }   

    float a = m[0][0];
    float b = m[0][1];
    float c = m[0][2];
    float d = m[1][0];
    float e = m[1][1];
    float f = m[1][2];
    float g = m[2][0];
    float h = m[2][1];
    float i = m[2][2];

    float coeff = (float) a*(e*i-f*h) - b*(d*i-f*g) + c*(d*h-e*g);
    coeff = (float) (1 / coeff);

    //printf("coeff = %7.6f\n", coeff);    

    o[0][0] = coeff * (e*i-f*h);
    o[0][1] = coeff * (c*h-b*i);
    o[0][2] = coeff * (b*f-c*e);
    o[1][0] = coeff * (f*g-d*i);
    o[1][1] = coeff * (a*i-c*g);
    o[1][2] = coeff * (c*d-a*f);
    o[2][0] = coeff * (d*h-e*g);
    o[2][1] = coeff * (b*g-a*h);
    o[2][2] = coeff * (a*e-b*d);

    return o;
}

// Determinant of a 3x3 matrix
float matrixDet_3x3( std::vector<std::vector<float>> m) {
    return m[0][0] * ( m[1][1] * m[2][2]  -  m[2][1] * m[1][2])
         - m[0][1] * ( m[1][0] * m[2][2]  -  m[2][0] * m[1][2])
         + m[0][2] * ( m[1][0] * m[2][1]  -  m[2][0] * m[1][1]);
}

