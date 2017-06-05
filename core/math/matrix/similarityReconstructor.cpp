#include "similarityReconstructor.h"
#include "levenmarq.h"

namespace corecvs {

SimilarityReconstructor::SimilarityReconstructor() :
    trace(false)
{
}

void SimilarityReconstructor::addPoint2PointConstraint(const Vector3dd &from, const Vector3dd &to)
{
    p2p.push_back(Correspondence3D(from,to));
}

void SimilarityReconstructor::addPoint2PointConstraint(double fromX, double fromY, double fromZ, double toX, double toY, double toZ)
{
    addPoint2PointConstraint(Vector3dd(fromX, fromY, fromZ), Vector3dd(toX, toY, toZ));
}

void SimilarityReconstructor::reset(void)
{
    p2p.clear();
}


Matrix44 Similarity::toMatrix() const
{
    return Matrix44::Shift(shiftR) * Matrix44::Scale(scaleR) * rotation.toMatrix() * Matrix44::Scale(1.0 / scaleL) * Matrix44::Shift(-shiftL);
}

Vector3dd Similarity::transform(const Vector3dd &point) const
{
    return (rotation * ((point - shiftL) / scaleL)) * scaleR + shiftR;
}

double Similarity::getScale()
{
    return scaleL / scaleR;
}

double Similarity::getInvScale()
{
    return scaleR / scaleL;
}

Vector3dd Similarity::getShift()
{
    Vector3dd shift = (shiftR / scaleR);
    shift = rotation.conjugated() * shift;
    shift = shift * scaleL;
    shift -= shiftL;

    return shift;
}

Matrix33 Similarity::getRotation()
{
    return rotation.toMatrix();
}

#if 1

#define DECOMPOSE_EPSILON	0.000001
#define RANKDECOMPOSE(a, b, c, x, y, z)      \
    if ((x) < (y))                   \
    {                               \
        if ((y) < (z))               \
        {                           \
            (a) = 2;                \
            (b) = 1;                \
            (c) = 0;                \
        }                           \
        else                        \
        {                           \
            (a) = 1;                \
                                    \
            if ((x) < (z))           \
            {                       \
                (b) = 2;            \
                (c) = 0;            \
            }                       \
            else                    \
            {                       \
                (b) = 0;            \
                (c) = 2;            \
            }                       \
        }                           \
    }                               \
    else                            \
    {                               \
        if ((x) < (z))               \
        {                           \
            (a) = 2;                \
            (b) = 0;                \
            (c) = 1;                \
        }                           \
        else                        \
        {                           \
            (a) = 0;                \
                                    \
            if ((y) < (z))           \
            {                       \
                (b) = 2;            \
                (c) = 1;            \
            }                       \
            else                    \
            {                       \
                (b) = 1;            \
                (c) = 2;            \
            }                       \
        }                           \
    }            

bool			DecomposeTRS(const Matrix44& matrix, Vector3dd& scale, Vector3dd& translate, Quaternion& qRotate)
{
	double det;
	double *pfScales;
	Vector4dd *ppvBasis[3];
	Vector4dd matTemp[4];

	static const Vector4dd x4 = { 1, 0, 0, 0 };
	static const Vector4dd y4 = { 0, 1, 0, 0 };
	static const Vector4dd z4 = { 0, 0, 1, 0 };
	static const Vector4dd w4 = { 0, 0, 0, 1 };

	uint a, b, c;
	static const Vector4dd *pvCanonicalBasis[3] = {
		&x4,
		&y4,
		&z4
	};

	const Matrix44 m = matrix.transposed();

	// Get the translation
	translate = matrix.translationPart();

	matTemp[0] = m.row(0);
	matTemp[1] = m.row(1);
	matTemp[2] = m.row(2);
	matTemp[3] = w4;

	ppvBasis[0] = &matTemp[0];
	ppvBasis[1] = &matTemp[1];
	ppvBasis[2] = &matTemp[2];

	pfScales = scale.element;

	pfScales[0] = sqrt(ppvBasis[0]->x * ppvBasis[0]->x + ppvBasis[0]->y * ppvBasis[0]->y + ppvBasis[0]->z * ppvBasis[0]->z);
	pfScales[1] = sqrt(ppvBasis[1]->x * ppvBasis[1]->x + ppvBasis[1]->y * ppvBasis[1]->y + ppvBasis[1]->z * ppvBasis[1]->z);
	pfScales[2] = sqrt(ppvBasis[2]->x * ppvBasis[2]->x + ppvBasis[2]->y * ppvBasis[2]->y + ppvBasis[2]->z * ppvBasis[2]->z);
	
	RANKDECOMPOSE(a, b, c, pfScales[0], pfScales[1], pfScales[2])
	if (pfScales[a] < DECOMPOSE_EPSILON)
	{
		*ppvBasis[a] = *pvCanonicalBasis[a];
	}

	Vector3dd* pv3 = (Vector3dd*)ppvBasis[a];
	*ppvBasis[a] = Vector4dd(pv3->normalised(), 0.0f);

	if (pfScales[b] < DECOMPOSE_EPSILON)
	{
		uint aa, bb, cc;
		double fAbsX, fAbsY, fAbsZ;

		fAbsX = abs(ppvBasis[a]->x);
		fAbsY = abs(ppvBasis[a]->y);
		fAbsZ = abs(ppvBasis[a]->z);

		RANKDECOMPOSE(aa, bb, cc, fAbsX, fAbsY, fAbsZ)

		*ppvBasis[b] = Float4(cross(Vector3dd((double*)ppvBasis[a]->element) ^ Vector3dd((double*)pvCanonicalBasis[cc])));
	}

	pv3 = (float3*)ppvBasis[b];
	*ppvBasis[b] = Float4(Normalize(*pv3));

	if (pfScales[c] < DECOMPOSE_EPSILON)
	{
		*ppvBasis[c] = Float4(Cross(Float3(*ppvBasis[a]), Float3(*ppvBasis[b])));
	}

	pv3 = (float3*)ppvBasis[c];
	*ppvBasis[c] = Float4(Normalize(*pv3));

	fDet = Determinant(Matrix44(matTemp[0], matTemp[1], matTemp[2], matTemp[3]));

	// use Kramer's rule to check for  handedness of coordinate system
	if (fDet < 0.0f)
	{
		// switch coordinate system by negating the scale and inverting the basis vector on the x-axis
		pfScales[a] = -pfScales[a];
		*ppvBasis[a] = *ppvBasis[a] * -1.0f;

		fDet = -fDet;
	}

	fDet -= 1.0f;
	fDet *= fDet;

	if (DECOMPOSE_EPSILON < fDet)
	{
		//		Non-SRT matrix encountered
		return false;
	}

	// generate the quaternion from the matrix
	qRotate = Quaternion::FromMatrix(Matrix44(matTemp[0], matTemp[1], matTemp[2], matTemp[3]).transposed().topLeft33());
	return true;
}

#undef DECOMPOSE_EPSILON
#undef RANKDECOMPOSE

/**
    X -> AX  - in space 1
	
	Similarity maps space1 to space2

	SX -  position in space2
	SAX - transformed position in space2

	this funtion returns B : BSX = SAX
	in operator form B = SAS^-1

 **/
Affine3DQ Similarity::transform(const Affine3DQ &A)
{
	Matrix44 result =
		Matrix44::Shift(shiftR) * Matrix44::Scale(scaleR) * rotation.toMatrix() * Matrix44::Scale(1.0 / scaleL) * Matrix44::Shift(-shiftL) * // S
		corecvs::Matrix44::Shift(A.shift[0], A.shift[1], A.shift[2]) * corecvs::Matrix44(A.rotor.toMatrix())
		* Matrix44::Shift(shiftL) *  Matrix44::Scale(scaleL) * rotation.conjugated.toMatrix() * Matrix44::Scale(1.0 / scaleR) * Matrix44::Shift(-shiftR);

    return Affi
}

#endif

void Similarity::fillFunctionInput(double in[])
{
    double scale    = getScale();
    Vector3dd shift = getShift();

    in[SHIFT_X] = -shift.x();
    in[SHIFT_Y] = -shift.y();
    in[SHIFT_Z] = -shift.z();
    in[SCALE]   = scale;
    in[ROTATION_X] = rotation.x();
    in[ROTATION_Y] = rotation.y();
    in[ROTATION_Z] = rotation.z();
    in[ROTATION_T] = rotation.t();
}


Similarity Similarity::getInterpolated(double t)
{
    Similarity toReturn;

    using corecvs::lerp;

    toReturn.scaleL = lerp(1.0, scaleL, t);
    toReturn.scaleR = lerp(1.0, scaleR, t);

    toReturn.shiftL = lerp(Vector3dd(0.0), shiftL, t);
    toReturn.shiftR = lerp(Vector3dd(0.0), shiftR, t);

    toReturn.rotation = Quaternion::slerp(Quaternion::RotationIdentity(), rotation, t);

    return toReturn;
}

Similarity Similarity::inverted()
{
    Similarity toReturn;
    toReturn.shiftL = shiftR;
    toReturn.shiftR = shiftL;
    toReturn.scaleL = scaleR;
    toReturn.scaleR = scaleL;

    toReturn.rotation = rotation.conjugated();


    return toReturn;
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

    if (trace)
        cout << "Starting optimization with " << p2p.size() << " constraints" << endl;

    for (unsigned  i = 0; i < p2p.size(); i++)
    {
        if (trace)
            cout << p2p[i].start << " ==> " << p2p[i].end << endl;

        lmean += p2p[i].start;
        lmeansq += (p2p[i].start) * (p2p[i].start);

        rmean += p2p[i].end;
        rmeansq += (p2p[i].end) * (p2p[i].end);
    }

    lmean    /= p2p.size();
    lmeansq  /= p2p.size();
    rmean    /= p2p.size();
    rmeansq  /= p2p.size();

    Vector3dd lsqmean = lmean * lmean;
    Vector3dd rsqmean = rmean * rmean;

    double lscaler = sqrt((lmeansq - lsqmean).sumAllElements());
    double rscaler = sqrt((rmeansq - rsqmean).sumAllElements());

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

    Matrix NA(N);
    DiagonalMatrix D(4);
    Matrix VT(4, 4);
    Matrix::jacobi(&NA, &D, &VT, NULL);

    double max = 0;
    int maxi = 0;
    for (int i = 0; i < 4; i++)
    {
        if (D.a(i) > max) {
            max = D.a(i);
            maxi = i;
        }
    }

    if (trace)
    {
        cout << "Cross Correlation:\n" << S;
        cout << "N Matrix         :\n" << N;
        cout << "Decomposition:\n" << NA << endl;
        cout << D << endl;
        cout << VT << endl;
        cout << "Maximum eigenvalue:" << max << " at row/column " << maxi << endl;
    }

    /* make an eigenvalue cheak */
    FixedVector<double, 4> eigen;
    eigen[0] = VT.a(0, maxi);
    eigen[1] = VT.a(1, maxi);
    eigen[2] = VT.a(2, maxi);
    eigen[3] = VT.a(3, maxi);

    Quaternion rotation(eigen[1], eigen[2], eigen[3], eigen[0]);

    if (trace)
    {
        cout << "Checking eigenvector:" <<  eigen << " " << eigen * D.a(0,maxi) << "  "  << (N * eigen) << endl;
        cout << "Quaternion:" << rotation << endl;
    }

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
    LMfit.maxIterations = 100;
    LMfit.traceProgress = false;

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
        Correspondence3D &corr = p2p[i];
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

void SimilarityReconstructor::reportInputQuality()
{
    /* Check all tha pairs of correspondance */

    //double meanScaler = 1.0; /* arithmetic average? Really? */

    cout << "Pairs ratio (expected to be constant):" << endl;
    for (size_t i = 0; i < p2p.size(); i++)
    {
        for (size_t j = i + 1; j < p2p.size(); j++)
        {
            double d1 = (p2p[i].start - p2p[j].start).l2Metric();
            double d2 = (p2p[i].end   - p2p[j].end  ).l2Metric();

            double scaler = d1 / d2;
            cout << i << " " << j << " " << scaler << endl;
        }
    }
}


ostream &operator << (ostream &out, const Similarity &reconstructor)
{
    out << "Shift Left  by: "  << reconstructor.shiftL << endl;
    out << "Scale Left  by: "  << reconstructor.scaleL << " (" << (1 / reconstructor.scaleL) << ")"<< endl;

    out << "Shift Right by: "  << reconstructor.shiftR << endl;
    out << "Scale Right by: "  << reconstructor.scaleR << " (" << (1 / reconstructor.scaleR) << ")" << endl;

    out << "Quaternion:" << reconstructor.rotation << endl;
    double angle = reconstructor.rotation.normalised().getAngle();
    out << "Rotate by: " << angle << " (" << radToDeg(angle) << "deg) around " << reconstructor.rotation.getAxis() << endl;
    Quaternion rot = reconstructor.rotation;
    rot = -rot;
    out << "Rotate by: " << rot.normalised().getAngle() << " around " << rot.getAxis() << endl;
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
        Correspondence3D &corr = reconstructor->p2p[i];
        out[i] = sqrt((corr.end - trans * corr.start).sumAllElementsSq());
    }
}


} // namespace corecvs
