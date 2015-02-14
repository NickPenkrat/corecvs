#include "similarityReconstructor.h"
#include "levenmarq.h"

namespace corecvs {

SimilarityReconstructor::SimilarityReconstructor()
{
}

void SimilarityReconstructor::addPoint2PointConstraint(const Vector3dd &from, const Vector3dd &to)
{
    p2p.push_back(Correspondance3D(from,to));
}

void SimilarityReconstructor::reset(void)
{
    p2p.clear();
}


Matrix44 Similarity::toMatrix() const
{
    return Matrix44::Shift(shiftR) * Matrix44::Scale(scaleR) * rotation.toMatrix() * Matrix44::Scale(1.0 / scaleL) * Matrix44::Shift(-shiftL);
}

void Similarity::fillFunctionInput(double in[])
{
    double scale    = scaleL / scaleR;
    Vector3dd shift = (shiftR / scaleR);
    shift = rotation.conjugated() * shift;
    shift = shift * scaleL;
    shift -= shiftL;

    in[SHIFT_X] = -shift.x();
    in[SHIFT_Y] = -shift.y();
    in[SHIFT_Z] = -shift.z();
    in[SCALE]   = scale;
    in[ROTATION_X] = rotation.x();
    in[ROTATION_Y] = rotation.y();
    in[ROTATION_Z] = rotation.z();
    in[ROTATION_T] = rotation.t();
}

/**
 *  Move similar code from  SimilarityReconstructor and HomographyReconstructor
 *
 *  http://people.csail.mit.edu/bkph/papers/Absolute_Orientation.pdf
 *
 **/
Similarity SimilarityReconstructor::getBestSimilarity()
{
    Similarity result;
    Vector3dd lmean(0.0);
    Vector3dd lmeansq(0.0);
    Vector3dd rmean(0.0);
    Vector3dd rmeansq(0.0);

    int lcount = 0;
    int rcount = 0;

    for (unsigned  i = 0; i < p2p.size(); i++)
    {
        lmean += p2p[i].start;
        lmeansq += (p2p[i].start) * (p2p[i].start);

        rmean += p2p[i].end;
        rmeansq += (p2p[i].end) * (p2p[i].end);

        lcount++;
        rcount++;
    }

    lmean /= lcount;
    lmeansq  /= lcount;
    rmean  /= rcount;
    rmeansq  /= rcount;

    Vector3dd lsqmean = lmean * lmean;
    Vector3dd rsqmean = rmean * rmean;

    double lscaler = sqrt (lmeansq.x() - lsqmean.x()  + lmeansq.y() - lsqmean.y());
    double rscaler = sqrt (rmeansq.x() - rsqmean.x()  + rmeansq.y() - rsqmean.y());

    /**
     *  so we need to shift first set to 0
     *  scale and shift back, so first element need to be mutiplied by scale
     **/
    result.scaleL = lscaler;
    result.scaleR = rscaler;
    result.shiftL = lmean;
    result.shiftR = rmean;

    /* Now we estimate the rotation */
    Matrix33 S(0.0);
    for (unsigned  i = 0; i < p2p.size(); i++)
    {
        S += Matrix33::VectorByVector((p2p[i].start - lmean) / lscaler, (p2p[i].end - rmean) / rscaler);
    }

    enum {X = 0, Y = 1, Z = 2};

    Matrix44 N = Matrix44(
        S(X, X) + S(Y, Y) + S(Z, Z),        S(Y,Z) - S(Z,Y)       ,        S(Z,X) - S(X,Z)         ,        S(X,Y) - S(Y,X)         ,
            S(Y,Z) - S(Z,Y)          , S(X, X) - S(Y, Y) - S(Z, Z),        S(X,Y) + S(Y,X)         ,        S(Z,X) + S(X,Z)         ,
            S(Z,X) - S(X,Z)          ,        S(X,Y) + S(Y,X)       , - S(X, X) + S(Y, Y) - S(Z, Z),        S(Z,Y) + S(Y,Z)         ,
            S(X,Y) - S(Y,X)          ,        S(Z,X) + S(X,Z)       ,         S(Z,Y) + S(Y,Z)        , - S(X, X) - S(Y, Y) + S(Z, Z)
    );

    cout << "Cross Correlation:\n" << S;
    cout << "N Matrix         :\n" << N;

    Matrix NA(N);
    DiagonalMatrix D(4);
    Matrix VT(4, 4);
    Matrix::jacobi(&NA, &D, &VT, NULL);

    cout << "Decomposition:\n" << NA << endl;
    cout << D << endl;
    cout << VT << endl;

    int max = 0;
    int maxi = 0;
    for (int i = 0; i < 4; i++)
    {
        if (D.a(0, i) > max) {
            max = D.a(0, i);
            maxi = i;
        }
    }
    maxi = 0;

    /* make an eigenvalue cheak */
    int row = 0;
    FixedVector<double, 4> eigen;
    eigen[0] = VT.a(0, row);
    eigen[1] = VT.a(1, row);
    eigen[2] = VT.a(2, row);
    eigen[3] = VT.a(3, row);

    cout << eigen << " " << eigen * D.a(0,row) << "  "  << (N * eigen) << endl;

    Quaternion rotation(eigen[1], eigen[2], eigen[3], eigen[0]);

    cout << "Quaternion:" << rotation << endl;

    result.rotation = rotation;
    return result;
}

Similarity SimilarityReconstructor::getBestSimilarityLM(Similarity &firstGuess)
{
    vector<double> guess(Similarity::PARAM_NUMBER);
    firstGuess.fillFunctionInput(&guess[0]);

    CostFunction F(this);
    NormalizeFunction N;
    LevenbergMarquardt LMfit;

    LMfit.f = &F;
    LMfit.normalisation = &N;
    LMfit.maxIterations = 1000;

    vector<double> output(1);
    output[0] = 0.0;

    vector<double> optInput = LMfit.fit(guess, output);

    return Similarity(&optInput[0]);
}

Similarity SimilarityReconstructor::getBestSimilarityLMN(Similarity &firstGuess)
{
    vector<double> guess(Similarity::PARAM_NUMBER);
    firstGuess.fillFunctionInput(&guess[0]);

    CostFunctionToN F(this);
    NormalizeFunction N;
    LevenbergMarquardt LMfit;

    LMfit.f = &F;
    LMfit.normalisation = &N;
    LMfit.maxIterations = 1000;

    vector<double> output(F.outputs);
    memset(&output[0], 0, sizeof(double) * F.outputs);

    vector<double> optInput = LMfit.fit(guess, output);

    return Similarity(&optInput[0]);
}

double SimilarityReconstructor::getCostFunction(const Similarity &input)
{
    double diff = 0.0;
    Matrix44 trans = input.toMatrix();
    for (unsigned  i = 0; i < p2p.size(); i++)
    {
        Correspondance3D &corr = p2p[i];
        diff += (corr.end - trans * corr.start).sumAllElementsSq();
    }
    if (p2p.size() != 0) {
        return sqrt(diff / p2p.size());
    }
    return 0.0;
}

SimilarityReconstructor::~SimilarityReconstructor()
{

}


ostream &operator << (ostream &out, const Similarity &reconstructor)
{
    out << "Shift Left  by: "  << reconstructor.shiftL << endl;
    out << "Scale Left  by: "  << reconstructor.scaleL << endl;

    out << "Shift Right by: "  << reconstructor.shiftR << endl;
    out << "Scale Right by: "  << reconstructor.scaleR << endl;

    out << "Quaternion:" << reconstructor.rotation << endl;
    out << "Rotate by: " << reconstructor.rotation.getAngle() << " around " << reconstructor.rotation.getAxis() << endl;
    Quaternion rot = reconstructor.rotation;
    rot = -rot;
    out << "Rotate by: " << rot.getAngle() << " around " << rot.getAxis() << endl;
    return  out;
}

void SimilarityReconstructor::CostFunction::operator()(const double in[], double out[])
{
    out[0] = reconstructor->getCostFunction(Similarity(in));
}

void SimilarityReconstructor::NormalizeFunction::operator()(const double in[], double out[])
{
    Similarity s(in);
    s.rotation.normalise();
    s.fillFunctionInput(out);
}

void SimilarityReconstructor::CostFunctionToN::operator()(const double in[], double out[])
{
    Similarity input(in);
    Matrix44 trans = input.toMatrix();
    for (unsigned i = 0; i < reconstructor->p2p.size(); i++)
    {
        Correspondance3D &corr = reconstructor->p2p[i];
        out[i] = sqrt((corr.end - trans * corr.start).sumAllElementsSq());
    }
}



};
