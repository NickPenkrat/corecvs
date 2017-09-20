#include <vector>
#include "correspondenceList.h"
#include "essentialEstimator.h"

using namespace std;
namespace corecvs {

Matrix derivative(const double in[], const vector<Correspondence *> *samples) {
    Matrix result(samples->size(), EssentialEstimator::CostFunctionBase::VECTOR_SIZE);
    double Qx = in[EssentialEstimator::CostFunctionBase::ROTATION_Q_X]; 
    double Qy = in[EssentialEstimator::CostFunctionBase::ROTATION_Q_Y]; 
    double Qz = in[EssentialEstimator::CostFunctionBase::ROTATION_Q_Z]; 
    double Qt = in[EssentialEstimator::CostFunctionBase::ROTATION_Q_T]; 
    double Tx = in[EssentialEstimator::CostFunctionBase::TRANSLATION_X]; 
    double Ty = in[EssentialEstimator::CostFunctionBase::TRANSLATION_Y]; 
    double Tz = in[EssentialEstimator::CostFunctionBase::TRANSLATION_Z]; 
    double a00 = (0-Tz)*(Qx*(Qy*(2/sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt)))+Qt*(Qz*(2/sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt))))+Ty*(Qx*(Qz*(2/sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt)))-Qt*(Qy*(2/sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt))));

     double a00dQx = (0-Tz)*((Qy*(2/sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt))+Qx*(Qy*((0-2*(0.5*((1.0 / sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt))*(Qx+Qx))))/(sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt)*sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt)))))+Qt*(Qz*((0-2*(0.5*((1.0 / sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt))*(Qx+Qx))))/(sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt)*sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt)))))+Ty*((Qz*(2/sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt))+Qx*(Qz*((0-2*(0.5*((1.0 / sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt))*(Qx+Qx))))/(sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt)*sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt)))))-Qt*(Qy*((0-2*(0.5*((1.0 / sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt))*(Qx+Qx))))/(sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt)*sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt)))));
     double a00dQy = (0-Tz)*(Qx*(2/sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt)+Qy*((0-2*(0.5*((1.0 / sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt))*(Qy+Qy))))/(sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt)*sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt))))+Qt*(Qz*((0-2*(0.5*((1.0 / sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt))*(Qy+Qy))))/(sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt)*sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt)))))+Ty*(Qx*(Qz*((0-2*(0.5*((1.0 / sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt))*(Qy+Qy))))/(sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt)*sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt))))-Qt*(2/sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt)+Qy*((0-2*(0.5*((1.0 / sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt))*(Qy+Qy))))/(sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt)*sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt)))));
     double a00dQz = (0-Tz)*(Qx*(Qy*((0-2*(0.5*((1.0 / sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt))*(Qz+Qz))))/(sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt)*sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt))))+Qt*(2/sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt)+Qz*((0-2*(0.5*((1.0 / sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt))*(Qz+Qz))))/(sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt)*sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt)))))+Ty*(Qx*(2/sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt)+Qz*((0-2*(0.5*((1.0 / sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt))*(Qz+Qz))))/(sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt)*sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt))))-Qt*(Qy*((0-2*(0.5*((1.0 / sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt))*(Qz+Qz))))/(sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt)*sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt)))));
     double a00dQt = (0-Tz)*(Qx*(Qy*((0-2*(0.5*((1.0 / sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt))*(Qt+Qt))))/(sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt)*sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt))))+(Qz*(2/sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt))+Qt*(Qz*((0-2*(0.5*((1.0 / sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt))*(Qt+Qt))))/(sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt)*sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt))))))+Ty*(Qx*(Qz*((0-2*(0.5*((1.0 / sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt))*(Qt+Qt))))/(sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt)*sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt))))-(Qy*(2/sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt))+Qt*(Qy*((0-2*(0.5*((1.0 / sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt))*(Qt+Qt))))/(sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt)*sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt))))));
     double a00dTx = 0;
     double a00dTy = Qx*(Qz*(2/sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt)))-Qt*(Qy*(2/sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt)));
     double a00dTz = (-1)*(Qx*(Qy*(2/sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt)))+Qt*(Qz*(2/sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt))));

    double a01 = (0-Tz)*(1-(Qx*(Qx*(2/sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt)))+Qz*(Qz*(2/sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt)))))+Ty*(Qy*(Qz*(2/sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt)))+Qt*(Qx*(2/sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt))));

     double a01dQx = (0-Tz)*(0-((Qx*(2/sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt))+Qx*(2/sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt)+Qx*((0-2*(0.5*((1.0 / sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt))*(Qx+Qx))))/(sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt)*sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt)))))+Qz*(Qz*((0-2*(0.5*((1.0 / sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt))*(Qx+Qx))))/(sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt)*sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt))))))+Ty*(Qy*(Qz*((0-2*(0.5*((1.0 / sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt))*(Qx+Qx))))/(sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt)*sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt))))+Qt*(2/sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt)+Qx*((0-2*(0.5*((1.0 / sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt))*(Qx+Qx))))/(sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt)*sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt)))));
     double a01dQy = (0-Tz)*(0-(Qx*(Qx*((0-2*(0.5*((1.0 / sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt))*(Qy+Qy))))/(sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt)*sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt))))+Qz*(Qz*((0-2*(0.5*((1.0 / sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt))*(Qy+Qy))))/(sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt)*sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt))))))+Ty*((Qz*(2/sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt))+Qy*(Qz*((0-2*(0.5*((1.0 / sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt))*(Qy+Qy))))/(sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt)*sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt)))))+Qt*(Qx*((0-2*(0.5*((1.0 / sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt))*(Qy+Qy))))/(sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt)*sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt)))));
     double a01dQz = (0-Tz)*(0-(Qx*(Qx*((0-2*(0.5*((1.0 / sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt))*(Qz+Qz))))/(sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt)*sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt))))+(Qz*(2/sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt))+Qz*(2/sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt)+Qz*((0-2*(0.5*((1.0 / sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt))*(Qz+Qz))))/(sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt)*sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt)))))))+Ty*(Qy*(2/sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt)+Qz*((0-2*(0.5*((1.0 / sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt))*(Qz+Qz))))/(sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt)*sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt))))+Qt*(Qx*((0-2*(0.5*((1.0 / sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt))*(Qz+Qz))))/(sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt)*sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt)))));
     double a01dQt = (0-Tz)*(0-(Qx*(Qx*((0-2*(0.5*((1.0 / sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt))*(Qt+Qt))))/(sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt)*sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt))))+Qz*(Qz*((0-2*(0.5*((1.0 / sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt))*(Qt+Qt))))/(sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt)*sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt))))))+Ty*(Qy*(Qz*((0-2*(0.5*((1.0 / sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt))*(Qt+Qt))))/(sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt)*sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt))))+(Qx*(2/sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt))+Qt*(Qx*((0-2*(0.5*((1.0 / sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt))*(Qt+Qt))))/(sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt)*sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt))))));
     double a01dTx = 0;
     double a01dTy = Qy*(Qz*(2/sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt)))+Qt*(Qx*(2/sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt)));
     double a01dTz = (-1)*(1-(Qx*(Qx*(2/sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt)))+Qz*(Qz*(2/sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt)))));

    double a02 = (0-Tz)*(Qy*(Qz*(2/sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt)))-Qt*(Qx*(2/sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt))))+Ty*(1-(Qx*(Qx*(2/sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt)))+Qy*(Qy*(2/sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt)))));

     double a02dQx = (0-Tz)*(Qy*(Qz*((0-2*(0.5*((1.0 / sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt))*(Qx+Qx))))/(sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt)*sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt))))-Qt*(2/sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt)+Qx*((0-2*(0.5*((1.0 / sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt))*(Qx+Qx))))/(sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt)*sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt)))))+Ty*(0-((Qx*(2/sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt))+Qx*(2/sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt)+Qx*((0-2*(0.5*((1.0 / sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt))*(Qx+Qx))))/(sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt)*sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt)))))+Qy*(Qy*((0-2*(0.5*((1.0 / sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt))*(Qx+Qx))))/(sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt)*sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt))))));
     double a02dQy = (0-Tz)*((Qz*(2/sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt))+Qy*(Qz*((0-2*(0.5*((1.0 / sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt))*(Qy+Qy))))/(sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt)*sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt)))))-Qt*(Qx*((0-2*(0.5*((1.0 / sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt))*(Qy+Qy))))/(sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt)*sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt)))))+Ty*(0-(Qx*(Qx*((0-2*(0.5*((1.0 / sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt))*(Qy+Qy))))/(sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt)*sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt))))+(Qy*(2/sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt))+Qy*(2/sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt)+Qy*((0-2*(0.5*((1.0 / sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt))*(Qy+Qy))))/(sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt)*sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt)))))));
     double a02dQz = (0-Tz)*(Qy*(2/sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt)+Qz*((0-2*(0.5*((1.0 / sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt))*(Qz+Qz))))/(sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt)*sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt))))-Qt*(Qx*((0-2*(0.5*((1.0 / sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt))*(Qz+Qz))))/(sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt)*sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt)))))+Ty*(0-(Qx*(Qx*((0-2*(0.5*((1.0 / sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt))*(Qz+Qz))))/(sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt)*sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt))))+Qy*(Qy*((0-2*(0.5*((1.0 / sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt))*(Qz+Qz))))/(sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt)*sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt))))));
     double a02dQt = (0-Tz)*(Qy*(Qz*((0-2*(0.5*((1.0 / sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt))*(Qt+Qt))))/(sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt)*sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt))))-(Qx*(2/sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt))+Qt*(Qx*((0-2*(0.5*((1.0 / sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt))*(Qt+Qt))))/(sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt)*sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt))))))+Ty*(0-(Qx*(Qx*((0-2*(0.5*((1.0 / sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt))*(Qt+Qt))))/(sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt)*sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt))))+Qy*(Qy*((0-2*(0.5*((1.0 / sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt))*(Qt+Qt))))/(sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt)*sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt))))));
     double a02dTx = 0;
     double a02dTy = 1-(Qx*(Qx*(2/sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt)))+Qy*(Qy*(2/sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt))));
     double a02dTz = (-1)*(Qy*(Qz*(2/sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt)))-Qt*(Qx*(2/sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt))));

    double a10 = Tz*(1-(Qy*(Qy*(2/sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt)))+Qz*(Qz*(2/sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt)))))+(0-Tx)*(Qx*(Qz*(2/sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt)))-Qt*(Qy*(2/sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt))));

     double a10dQx = Tz*(0-(Qy*(Qy*((0-2*(0.5*((1.0 / sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt))*(Qx+Qx))))/(sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt)*sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt))))+Qz*(Qz*((0-2*(0.5*((1.0 / sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt))*(Qx+Qx))))/(sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt)*sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt))))))+(0-Tx)*((Qz*(2/sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt))+Qx*(Qz*((0-2*(0.5*((1.0 / sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt))*(Qx+Qx))))/(sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt)*sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt)))))-Qt*(Qy*((0-2*(0.5*((1.0 / sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt))*(Qx+Qx))))/(sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt)*sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt)))));
     double a10dQy = Tz*(0-((Qy*(2/sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt))+Qy*(2/sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt)+Qy*((0-2*(0.5*((1.0 / sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt))*(Qy+Qy))))/(sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt)*sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt)))))+Qz*(Qz*((0-2*(0.5*((1.0 / sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt))*(Qy+Qy))))/(sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt)*sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt))))))+(0-Tx)*(Qx*(Qz*((0-2*(0.5*((1.0 / sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt))*(Qy+Qy))))/(sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt)*sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt))))-Qt*(2/sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt)+Qy*((0-2*(0.5*((1.0 / sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt))*(Qy+Qy))))/(sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt)*sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt)))));
     double a10dQz = Tz*(0-(Qy*(Qy*((0-2*(0.5*((1.0 / sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt))*(Qz+Qz))))/(sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt)*sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt))))+(Qz*(2/sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt))+Qz*(2/sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt)+Qz*((0-2*(0.5*((1.0 / sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt))*(Qz+Qz))))/(sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt)*sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt)))))))+(0-Tx)*(Qx*(2/sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt)+Qz*((0-2*(0.5*((1.0 / sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt))*(Qz+Qz))))/(sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt)*sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt))))-Qt*(Qy*((0-2*(0.5*((1.0 / sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt))*(Qz+Qz))))/(sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt)*sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt)))));
     double a10dQt = Tz*(0-(Qy*(Qy*((0-2*(0.5*((1.0 / sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt))*(Qt+Qt))))/(sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt)*sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt))))+Qz*(Qz*((0-2*(0.5*((1.0 / sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt))*(Qt+Qt))))/(sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt)*sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt))))))+(0-Tx)*(Qx*(Qz*((0-2*(0.5*((1.0 / sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt))*(Qt+Qt))))/(sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt)*sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt))))-(Qy*(2/sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt))+Qt*(Qy*((0-2*(0.5*((1.0 / sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt))*(Qt+Qt))))/(sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt)*sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt))))));
     double a10dTx = (-1)*(Qx*(Qz*(2/sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt)))-Qt*(Qy*(2/sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt))));
     double a10dTy = 0;
     double a10dTz = 1-(Qy*(Qy*(2/sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt)))+Qz*(Qz*(2/sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt))));

    double a11 = Tz*(Qx*(Qy*(2/sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt)))-Qt*(Qz*(2/sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt))))+(0-Tx)*(Qy*(Qz*(2/sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt)))+Qt*(Qx*(2/sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt))));

     double a11dQx = Tz*((Qy*(2/sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt))+Qx*(Qy*((0-2*(0.5*((1.0 / sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt))*(Qx+Qx))))/(sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt)*sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt)))))-Qt*(Qz*((0-2*(0.5*((1.0 / sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt))*(Qx+Qx))))/(sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt)*sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt)))))+(0-Tx)*(Qy*(Qz*((0-2*(0.5*((1.0 / sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt))*(Qx+Qx))))/(sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt)*sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt))))+Qt*(2/sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt)+Qx*((0-2*(0.5*((1.0 / sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt))*(Qx+Qx))))/(sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt)*sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt)))));
     double a11dQy = Tz*(Qx*(2/sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt)+Qy*((0-2*(0.5*((1.0 / sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt))*(Qy+Qy))))/(sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt)*sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt))))-Qt*(Qz*((0-2*(0.5*((1.0 / sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt))*(Qy+Qy))))/(sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt)*sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt)))))+(0-Tx)*((Qz*(2/sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt))+Qy*(Qz*((0-2*(0.5*((1.0 / sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt))*(Qy+Qy))))/(sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt)*sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt)))))+Qt*(Qx*((0-2*(0.5*((1.0 / sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt))*(Qy+Qy))))/(sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt)*sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt)))));
     double a11dQz = Tz*(Qx*(Qy*((0-2*(0.5*((1.0 / sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt))*(Qz+Qz))))/(sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt)*sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt))))-Qt*(2/sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt)+Qz*((0-2*(0.5*((1.0 / sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt))*(Qz+Qz))))/(sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt)*sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt)))))+(0-Tx)*(Qy*(2/sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt)+Qz*((0-2*(0.5*((1.0 / sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt))*(Qz+Qz))))/(sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt)*sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt))))+Qt*(Qx*((0-2*(0.5*((1.0 / sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt))*(Qz+Qz))))/(sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt)*sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt)))));
     double a11dQt = Tz*(Qx*(Qy*((0-2*(0.5*((1.0 / sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt))*(Qt+Qt))))/(sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt)*sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt))))-(Qz*(2/sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt))+Qt*(Qz*((0-2*(0.5*((1.0 / sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt))*(Qt+Qt))))/(sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt)*sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt))))))+(0-Tx)*(Qy*(Qz*((0-2*(0.5*((1.0 / sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt))*(Qt+Qt))))/(sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt)*sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt))))+(Qx*(2/sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt))+Qt*(Qx*((0-2*(0.5*((1.0 / sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt))*(Qt+Qt))))/(sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt)*sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt))))));
     double a11dTx = (-1)*(Qy*(Qz*(2/sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt)))+Qt*(Qx*(2/sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt))));
     double a11dTy = 0;
     double a11dTz = Qx*(Qy*(2/sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt)))-Qt*(Qz*(2/sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt)));

    double a12 = Tz*(Qx*(Qz*(2/sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt)))+Qt*(Qy*(2/sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt))))+(0-Tx)*(1-(Qx*(Qx*(2/sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt)))+Qy*(Qy*(2/sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt)))));

     double a12dQx = Tz*((Qz*(2/sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt))+Qx*(Qz*((0-2*(0.5*((1.0 / sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt))*(Qx+Qx))))/(sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt)*sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt)))))+Qt*(Qy*((0-2*(0.5*((1.0 / sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt))*(Qx+Qx))))/(sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt)*sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt)))))+(0-Tx)*(0-((Qx*(2/sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt))+Qx*(2/sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt)+Qx*((0-2*(0.5*((1.0 / sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt))*(Qx+Qx))))/(sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt)*sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt)))))+Qy*(Qy*((0-2*(0.5*((1.0 / sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt))*(Qx+Qx))))/(sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt)*sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt))))));
     double a12dQy = Tz*(Qx*(Qz*((0-2*(0.5*((1.0 / sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt))*(Qy+Qy))))/(sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt)*sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt))))+Qt*(2/sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt)+Qy*((0-2*(0.5*((1.0 / sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt))*(Qy+Qy))))/(sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt)*sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt)))))+(0-Tx)*(0-(Qx*(Qx*((0-2*(0.5*((1.0 / sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt))*(Qy+Qy))))/(sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt)*sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt))))+(Qy*(2/sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt))+Qy*(2/sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt)+Qy*((0-2*(0.5*((1.0 / sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt))*(Qy+Qy))))/(sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt)*sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt)))))));
     double a12dQz = Tz*(Qx*(2/sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt)+Qz*((0-2*(0.5*((1.0 / sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt))*(Qz+Qz))))/(sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt)*sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt))))+Qt*(Qy*((0-2*(0.5*((1.0 / sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt))*(Qz+Qz))))/(sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt)*sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt)))))+(0-Tx)*(0-(Qx*(Qx*((0-2*(0.5*((1.0 / sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt))*(Qz+Qz))))/(sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt)*sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt))))+Qy*(Qy*((0-2*(0.5*((1.0 / sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt))*(Qz+Qz))))/(sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt)*sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt))))));
     double a12dQt = Tz*(Qx*(Qz*((0-2*(0.5*((1.0 / sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt))*(Qt+Qt))))/(sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt)*sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt))))+(Qy*(2/sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt))+Qt*(Qy*((0-2*(0.5*((1.0 / sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt))*(Qt+Qt))))/(sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt)*sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt))))))+(0-Tx)*(0-(Qx*(Qx*((0-2*(0.5*((1.0 / sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt))*(Qt+Qt))))/(sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt)*sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt))))+Qy*(Qy*((0-2*(0.5*((1.0 / sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt))*(Qt+Qt))))/(sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt)*sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt))))));
     double a12dTx = (-1)*(1-(Qx*(Qx*(2/sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt)))+Qy*(Qy*(2/sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt)))));
     double a12dTy = 0;
     double a12dTz = Qx*(Qz*(2/sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt)))+Qt*(Qy*(2/sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt)));

    double a20 = (0-Ty)*(1-(Qy*(Qy*(2/sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt)))+Qz*(Qz*(2/sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt)))))+Tx*(Qx*(Qy*(2/sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt)))+Qt*(Qz*(2/sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt))));

     double a20dQx = (0-Ty)*(0-(Qy*(Qy*((0-2*(0.5*((1.0 / sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt))*(Qx+Qx))))/(sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt)*sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt))))+Qz*(Qz*((0-2*(0.5*((1.0 / sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt))*(Qx+Qx))))/(sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt)*sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt))))))+Tx*((Qy*(2/sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt))+Qx*(Qy*((0-2*(0.5*((1.0 / sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt))*(Qx+Qx))))/(sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt)*sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt)))))+Qt*(Qz*((0-2*(0.5*((1.0 / sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt))*(Qx+Qx))))/(sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt)*sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt)))));
     double a20dQy = (0-Ty)*(0-((Qy*(2/sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt))+Qy*(2/sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt)+Qy*((0-2*(0.5*((1.0 / sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt))*(Qy+Qy))))/(sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt)*sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt)))))+Qz*(Qz*((0-2*(0.5*((1.0 / sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt))*(Qy+Qy))))/(sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt)*sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt))))))+Tx*(Qx*(2/sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt)+Qy*((0-2*(0.5*((1.0 / sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt))*(Qy+Qy))))/(sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt)*sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt))))+Qt*(Qz*((0-2*(0.5*((1.0 / sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt))*(Qy+Qy))))/(sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt)*sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt)))));
     double a20dQz = (0-Ty)*(0-(Qy*(Qy*((0-2*(0.5*((1.0 / sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt))*(Qz+Qz))))/(sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt)*sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt))))+(Qz*(2/sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt))+Qz*(2/sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt)+Qz*((0-2*(0.5*((1.0 / sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt))*(Qz+Qz))))/(sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt)*sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt)))))))+Tx*(Qx*(Qy*((0-2*(0.5*((1.0 / sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt))*(Qz+Qz))))/(sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt)*sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt))))+Qt*(2/sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt)+Qz*((0-2*(0.5*((1.0 / sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt))*(Qz+Qz))))/(sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt)*sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt)))));
     double a20dQt = (0-Ty)*(0-(Qy*(Qy*((0-2*(0.5*((1.0 / sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt))*(Qt+Qt))))/(sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt)*sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt))))+Qz*(Qz*((0-2*(0.5*((1.0 / sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt))*(Qt+Qt))))/(sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt)*sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt))))))+Tx*(Qx*(Qy*((0-2*(0.5*((1.0 / sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt))*(Qt+Qt))))/(sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt)*sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt))))+(Qz*(2/sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt))+Qt*(Qz*((0-2*(0.5*((1.0 / sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt))*(Qt+Qt))))/(sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt)*sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt))))));
     double a20dTx = Qx*(Qy*(2/sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt)))+Qt*(Qz*(2/sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt)));
     double a20dTy = (-1)*(1-(Qy*(Qy*(2/sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt)))+Qz*(Qz*(2/sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt)))));
     double a20dTz = 0;

    double a21 = (0-Ty)*(Qx*(Qy*(2/sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt)))-Qt*(Qz*(2/sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt))))+Tx*(1-(Qx*(Qx*(2/sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt)))+Qz*(Qz*(2/sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt)))));

     double a21dQx = (0-Ty)*((Qy*(2/sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt))+Qx*(Qy*((0-2*(0.5*((1.0 / sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt))*(Qx+Qx))))/(sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt)*sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt)))))-Qt*(Qz*((0-2*(0.5*((1.0 / sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt))*(Qx+Qx))))/(sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt)*sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt)))))+Tx*(0-((Qx*(2/sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt))+Qx*(2/sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt)+Qx*((0-2*(0.5*((1.0 / sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt))*(Qx+Qx))))/(sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt)*sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt)))))+Qz*(Qz*((0-2*(0.5*((1.0 / sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt))*(Qx+Qx))))/(sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt)*sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt))))));
     double a21dQy = (0-Ty)*(Qx*(2/sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt)+Qy*((0-2*(0.5*((1.0 / sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt))*(Qy+Qy))))/(sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt)*sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt))))-Qt*(Qz*((0-2*(0.5*((1.0 / sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt))*(Qy+Qy))))/(sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt)*sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt)))))+Tx*(0-(Qx*(Qx*((0-2*(0.5*((1.0 / sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt))*(Qy+Qy))))/(sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt)*sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt))))+Qz*(Qz*((0-2*(0.5*((1.0 / sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt))*(Qy+Qy))))/(sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt)*sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt))))));
     double a21dQz = (0-Ty)*(Qx*(Qy*((0-2*(0.5*((1.0 / sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt))*(Qz+Qz))))/(sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt)*sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt))))-Qt*(2/sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt)+Qz*((0-2*(0.5*((1.0 / sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt))*(Qz+Qz))))/(sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt)*sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt)))))+Tx*(0-(Qx*(Qx*((0-2*(0.5*((1.0 / sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt))*(Qz+Qz))))/(sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt)*sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt))))+(Qz*(2/sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt))+Qz*(2/sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt)+Qz*((0-2*(0.5*((1.0 / sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt))*(Qz+Qz))))/(sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt)*sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt)))))));
     double a21dQt = (0-Ty)*(Qx*(Qy*((0-2*(0.5*((1.0 / sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt))*(Qt+Qt))))/(sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt)*sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt))))-(Qz*(2/sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt))+Qt*(Qz*((0-2*(0.5*((1.0 / sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt))*(Qt+Qt))))/(sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt)*sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt))))))+Tx*(0-(Qx*(Qx*((0-2*(0.5*((1.0 / sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt))*(Qt+Qt))))/(sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt)*sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt))))+Qz*(Qz*((0-2*(0.5*((1.0 / sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt))*(Qt+Qt))))/(sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt)*sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt))))));
     double a21dTx = 1-(Qx*(Qx*(2/sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt)))+Qz*(Qz*(2/sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt))));
     double a21dTy = (-1)*(Qx*(Qy*(2/sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt)))-Qt*(Qz*(2/sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt))));
     double a21dTz = 0;

    double a22 = (0-Ty)*(Qx*(Qz*(2/sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt)))+Qt*(Qy*(2/sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt))))+Tx*(Qy*(Qz*(2/sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt)))-Qt*(Qx*(2/sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt))));

     double a22dQx = (0-Ty)*((Qz*(2/sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt))+Qx*(Qz*((0-2*(0.5*((1.0 / sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt))*(Qx+Qx))))/(sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt)*sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt)))))+Qt*(Qy*((0-2*(0.5*((1.0 / sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt))*(Qx+Qx))))/(sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt)*sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt)))))+Tx*(Qy*(Qz*((0-2*(0.5*((1.0 / sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt))*(Qx+Qx))))/(sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt)*sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt))))-Qt*(2/sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt)+Qx*((0-2*(0.5*((1.0 / sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt))*(Qx+Qx))))/(sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt)*sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt)))));
     double a22dQy = (0-Ty)*(Qx*(Qz*((0-2*(0.5*((1.0 / sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt))*(Qy+Qy))))/(sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt)*sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt))))+Qt*(2/sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt)+Qy*((0-2*(0.5*((1.0 / sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt))*(Qy+Qy))))/(sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt)*sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt)))))+Tx*((Qz*(2/sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt))+Qy*(Qz*((0-2*(0.5*((1.0 / sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt))*(Qy+Qy))))/(sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt)*sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt)))))-Qt*(Qx*((0-2*(0.5*((1.0 / sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt))*(Qy+Qy))))/(sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt)*sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt)))));
     double a22dQz = (0-Ty)*(Qx*(2/sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt)+Qz*((0-2*(0.5*((1.0 / sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt))*(Qz+Qz))))/(sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt)*sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt))))+Qt*(Qy*((0-2*(0.5*((1.0 / sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt))*(Qz+Qz))))/(sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt)*sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt)))))+Tx*(Qy*(2/sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt)+Qz*((0-2*(0.5*((1.0 / sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt))*(Qz+Qz))))/(sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt)*sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt))))-Qt*(Qx*((0-2*(0.5*((1.0 / sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt))*(Qz+Qz))))/(sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt)*sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt)))));
     double a22dQt = (0-Ty)*(Qx*(Qz*((0-2*(0.5*((1.0 / sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt))*(Qt+Qt))))/(sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt)*sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt))))+(Qy*(2/sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt))+Qt*(Qy*((0-2*(0.5*((1.0 / sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt))*(Qt+Qt))))/(sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt)*sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt))))))+Tx*(Qy*(Qz*((0-2*(0.5*((1.0 / sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt))*(Qt+Qt))))/(sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt)*sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt))))-(Qx*(2/sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt))+Qt*(Qx*((0-2*(0.5*((1.0 / sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt))*(Qt+Qt))))/(sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt)*sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt))))));
     double a22dTx = Qy*(Qz*(2/sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt)))-Qt*(Qx*(2/sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt)));
     double a22dTy = (-1)*(Qx*(Qz*(2/sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt)))+Qt*(Qy*(2/sqrt(((Qx*Qx+Qy*Qy)+Qz*Qz)+Qt*Qt))));
     double a22dTz = 0;


   for (size_t i = 0; i < samples->size(); i++)
   {
        double startx = (*samples)[i]->start.x();
        double starty = (*samples)[i]->start.y();
        double endx   = (*samples)[i]->end.x();
        double endy   = (*samples)[i]->end.y();
   
         double v = ((((startx*a00+starty*a10)+a20)*endx+((startx*a01+starty*a11)+a21)*endy)+((startx*a02+starty*a12)+a22))/sqrt(((startx*a00+starty*a10)+a20)*((startx*a00+starty*a10)+a20)+((startx*a01+starty*a11)+a21)*((startx*a01+starty*a11)+a21));
         result.element(i,0) = (((((startx*a00dQx+starty*a10dQx)+a20dQx)*endx+((startx*a01dQx+starty*a11dQx)+a21dQx)*endy)+((startx*a02dQx+starty*a12dQx)+a22dQx))*sqrt(((startx*a00+starty*a10)+a20)*((startx*a00+starty*a10)+a20)+((startx*a01+starty*a11)+a21)*((startx*a01+starty*a11)+a21))-((((startx*a00+starty*a10)+a20)*endx+((startx*a01+starty*a11)+a21)*endy)+((startx*a02+starty*a12)+a22))*(0.5*((1.0 / sqrt(((startx*a00+starty*a10)+a20)*((startx*a00+starty*a10)+a20)+((startx*a01+starty*a11)+a21)*((startx*a01+starty*a11)+a21)))*((((startx*a00dQx+starty*a10dQx)+a20dQx)*((startx*a00+starty*a10)+a20)+((startx*a00+starty*a10)+a20)*((startx*a00dQx+starty*a10dQx)+a20dQx))+(((startx*a01dQx+starty*a11dQx)+a21dQx)*((startx*a01+starty*a11)+a21)+((startx*a01+starty*a11)+a21)*((startx*a01dQx+starty*a11dQx)+a21dQx))))))/(sqrt(((startx*a00+starty*a10)+a20)*((startx*a00+starty*a10)+a20)+((startx*a01+starty*a11)+a21)*((startx*a01+starty*a11)+a21))*sqrt(((startx*a00+starty*a10)+a20)*((startx*a00+starty*a10)+a20)+((startx*a01+starty*a11)+a21)*((startx*a01+starty*a11)+a21)));
         if (v < 0) result.element(i,0) = -result.element(i,0);
         result.element(i,1) = (((((startx*a00dQy+starty*a10dQy)+a20dQy)*endx+((startx*a01dQy+starty*a11dQy)+a21dQy)*endy)+((startx*a02dQy+starty*a12dQy)+a22dQy))*sqrt(((startx*a00+starty*a10)+a20)*((startx*a00+starty*a10)+a20)+((startx*a01+starty*a11)+a21)*((startx*a01+starty*a11)+a21))-((((startx*a00+starty*a10)+a20)*endx+((startx*a01+starty*a11)+a21)*endy)+((startx*a02+starty*a12)+a22))*(0.5*((1.0 / sqrt(((startx*a00+starty*a10)+a20)*((startx*a00+starty*a10)+a20)+((startx*a01+starty*a11)+a21)*((startx*a01+starty*a11)+a21)))*((((startx*a00dQy+starty*a10dQy)+a20dQy)*((startx*a00+starty*a10)+a20)+((startx*a00+starty*a10)+a20)*((startx*a00dQy+starty*a10dQy)+a20dQy))+(((startx*a01dQy+starty*a11dQy)+a21dQy)*((startx*a01+starty*a11)+a21)+((startx*a01+starty*a11)+a21)*((startx*a01dQy+starty*a11dQy)+a21dQy))))))/(sqrt(((startx*a00+starty*a10)+a20)*((startx*a00+starty*a10)+a20)+((startx*a01+starty*a11)+a21)*((startx*a01+starty*a11)+a21))*sqrt(((startx*a00+starty*a10)+a20)*((startx*a00+starty*a10)+a20)+((startx*a01+starty*a11)+a21)*((startx*a01+starty*a11)+a21)));
         if (v < 0) result.element(i,1) = -result.element(i,1);
         result.element(i,2) = (((((startx*a00dQz+starty*a10dQz)+a20dQz)*endx+((startx*a01dQz+starty*a11dQz)+a21dQz)*endy)+((startx*a02dQz+starty*a12dQz)+a22dQz))*sqrt(((startx*a00+starty*a10)+a20)*((startx*a00+starty*a10)+a20)+((startx*a01+starty*a11)+a21)*((startx*a01+starty*a11)+a21))-((((startx*a00+starty*a10)+a20)*endx+((startx*a01+starty*a11)+a21)*endy)+((startx*a02+starty*a12)+a22))*(0.5*((1.0 / sqrt(((startx*a00+starty*a10)+a20)*((startx*a00+starty*a10)+a20)+((startx*a01+starty*a11)+a21)*((startx*a01+starty*a11)+a21)))*((((startx*a00dQz+starty*a10dQz)+a20dQz)*((startx*a00+starty*a10)+a20)+((startx*a00+starty*a10)+a20)*((startx*a00dQz+starty*a10dQz)+a20dQz))+(((startx*a01dQz+starty*a11dQz)+a21dQz)*((startx*a01+starty*a11)+a21)+((startx*a01+starty*a11)+a21)*((startx*a01dQz+starty*a11dQz)+a21dQz))))))/(sqrt(((startx*a00+starty*a10)+a20)*((startx*a00+starty*a10)+a20)+((startx*a01+starty*a11)+a21)*((startx*a01+starty*a11)+a21))*sqrt(((startx*a00+starty*a10)+a20)*((startx*a00+starty*a10)+a20)+((startx*a01+starty*a11)+a21)*((startx*a01+starty*a11)+a21)));
         if (v < 0) result.element(i,2) = -result.element(i,2);
         result.element(i,3) = (((((startx*a00dQt+starty*a10dQt)+a20dQt)*endx+((startx*a01dQt+starty*a11dQt)+a21dQt)*endy)+((startx*a02dQt+starty*a12dQt)+a22dQt))*sqrt(((startx*a00+starty*a10)+a20)*((startx*a00+starty*a10)+a20)+((startx*a01+starty*a11)+a21)*((startx*a01+starty*a11)+a21))-((((startx*a00+starty*a10)+a20)*endx+((startx*a01+starty*a11)+a21)*endy)+((startx*a02+starty*a12)+a22))*(0.5*((1.0 / sqrt(((startx*a00+starty*a10)+a20)*((startx*a00+starty*a10)+a20)+((startx*a01+starty*a11)+a21)*((startx*a01+starty*a11)+a21)))*((((startx*a00dQt+starty*a10dQt)+a20dQt)*((startx*a00+starty*a10)+a20)+((startx*a00+starty*a10)+a20)*((startx*a00dQt+starty*a10dQt)+a20dQt))+(((startx*a01dQt+starty*a11dQt)+a21dQt)*((startx*a01+starty*a11)+a21)+((startx*a01+starty*a11)+a21)*((startx*a01dQt+starty*a11dQt)+a21dQt))))))/(sqrt(((startx*a00+starty*a10)+a20)*((startx*a00+starty*a10)+a20)+((startx*a01+starty*a11)+a21)*((startx*a01+starty*a11)+a21))*sqrt(((startx*a00+starty*a10)+a20)*((startx*a00+starty*a10)+a20)+((startx*a01+starty*a11)+a21)*((startx*a01+starty*a11)+a21)));
         if (v < 0) result.element(i,3) = -result.element(i,3);
         result.element(i,4) = (((((startx*a00dTx+starty*a10dTx)+a20dTx)*endx+((startx*a01dTx+starty*a11dTx)+a21dTx)*endy)+((startx*a02dTx+starty*a12dTx)+a22dTx))*sqrt(((startx*a00+starty*a10)+a20)*((startx*a00+starty*a10)+a20)+((startx*a01+starty*a11)+a21)*((startx*a01+starty*a11)+a21))-((((startx*a00+starty*a10)+a20)*endx+((startx*a01+starty*a11)+a21)*endy)+((startx*a02+starty*a12)+a22))*(0.5*((1.0 / sqrt(((startx*a00+starty*a10)+a20)*((startx*a00+starty*a10)+a20)+((startx*a01+starty*a11)+a21)*((startx*a01+starty*a11)+a21)))*((((startx*a00dTx+starty*a10dTx)+a20dTx)*((startx*a00+starty*a10)+a20)+((startx*a00+starty*a10)+a20)*((startx*a00dTx+starty*a10dTx)+a20dTx))+(((startx*a01dTx+starty*a11dTx)+a21dTx)*((startx*a01+starty*a11)+a21)+((startx*a01+starty*a11)+a21)*((startx*a01dTx+starty*a11dTx)+a21dTx))))))/(sqrt(((startx*a00+starty*a10)+a20)*((startx*a00+starty*a10)+a20)+((startx*a01+starty*a11)+a21)*((startx*a01+starty*a11)+a21))*sqrt(((startx*a00+starty*a10)+a20)*((startx*a00+starty*a10)+a20)+((startx*a01+starty*a11)+a21)*((startx*a01+starty*a11)+a21)));
         if (v < 0) result.element(i,4) = -result.element(i,4);
         result.element(i,5) = (((((startx*a00dTy+starty*a10dTy)+a20dTy)*endx+((startx*a01dTy+starty*a11dTy)+a21dTy)*endy)+((startx*a02dTy+starty*a12dTy)+a22dTy))*sqrt(((startx*a00+starty*a10)+a20)*((startx*a00+starty*a10)+a20)+((startx*a01+starty*a11)+a21)*((startx*a01+starty*a11)+a21))-((((startx*a00+starty*a10)+a20)*endx+((startx*a01+starty*a11)+a21)*endy)+((startx*a02+starty*a12)+a22))*(0.5*((1.0 / sqrt(((startx*a00+starty*a10)+a20)*((startx*a00+starty*a10)+a20)+((startx*a01+starty*a11)+a21)*((startx*a01+starty*a11)+a21)))*((((startx*a00dTy+starty*a10dTy)+a20dTy)*((startx*a00+starty*a10)+a20)+((startx*a00+starty*a10)+a20)*((startx*a00dTy+starty*a10dTy)+a20dTy))+(((startx*a01dTy+starty*a11dTy)+a21dTy)*((startx*a01+starty*a11)+a21)+((startx*a01+starty*a11)+a21)*((startx*a01dTy+starty*a11dTy)+a21dTy))))))/(sqrt(((startx*a00+starty*a10)+a20)*((startx*a00+starty*a10)+a20)+((startx*a01+starty*a11)+a21)*((startx*a01+starty*a11)+a21))*sqrt(((startx*a00+starty*a10)+a20)*((startx*a00+starty*a10)+a20)+((startx*a01+starty*a11)+a21)*((startx*a01+starty*a11)+a21)));
         if (v < 0) result.element(i,5) = -result.element(i,5);
         result.element(i,6) = (((((startx*a00dTz+starty*a10dTz)+a20dTz)*endx+((startx*a01dTz+starty*a11dTz)+a21dTz)*endy)+((startx*a02dTz+starty*a12dTz)+a22dTz))*sqrt(((startx*a00+starty*a10)+a20)*((startx*a00+starty*a10)+a20)+((startx*a01+starty*a11)+a21)*((startx*a01+starty*a11)+a21))-((((startx*a00+starty*a10)+a20)*endx+((startx*a01+starty*a11)+a21)*endy)+((startx*a02+starty*a12)+a22))*(0.5*((1.0 / sqrt(((startx*a00+starty*a10)+a20)*((startx*a00+starty*a10)+a20)+((startx*a01+starty*a11)+a21)*((startx*a01+starty*a11)+a21)))*((((startx*a00dTz+starty*a10dTz)+a20dTz)*((startx*a00+starty*a10)+a20)+((startx*a00+starty*a10)+a20)*((startx*a00dTz+starty*a10dTz)+a20dTz))+(((startx*a01dTz+starty*a11dTz)+a21dTz)*((startx*a01+starty*a11)+a21)+((startx*a01+starty*a11)+a21)*((startx*a01dTz+starty*a11dTz)+a21dTz))))))/(sqrt(((startx*a00+starty*a10)+a20)*((startx*a00+starty*a10)+a20)+((startx*a01+starty*a11)+a21)*((startx*a01+starty*a11)+a21))*sqrt(((startx*a00+starty*a10)+a20)*((startx*a00+starty*a10)+a20)+((startx*a01+starty*a11)+a21)*((startx*a01+starty*a11)+a21)));
         if (v < 0) result.element(i,6) = -result.element(i,6);
   }
   return result;
}
} // namespace
