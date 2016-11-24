#include "sparseMatrix.h"

#ifdef WITH_CUSPARSE
namespace corecvs
{

SparseMatrix::CUDAPromoter::CUDAPromoter(const SparseMatrix &m) : matrix(&m)
{
}

SparseMatrix::CUDAPromoter::TriangularPromotion::TriangularPromotion()
{
}

SparseMatrix::CUDAPromoter::TriangularPromotion::TriangularPromotion(SparseMatrix::CUDAPromoter::TriangularPromotion &&rhs)
     : bufferTrans(std::move(rhs.bufferTrans)),
       bufferNoTrans(std::move(rhs.bufferNoTrans)),
       policy(rhs.policy),
       descr(rhs.descr),
       handle(rhs.handle),
       infoTrans(rhs.infoTrans),
       infoNoTrans(rhs.infoNoTrans)
{
}
SparseMatrix::CUDAPromoter::TriangularPromotion& SparseMatrix::CUDAPromoter::TriangularPromotion::operator=(SparseMatrix::CUDAPromoter::TriangularPromotion &&rhs)
{
    std::swap(rhs.bufferTrans, bufferTrans);
    std::swap(rhs.bufferNoTrans, bufferNoTrans);
    std::swap(rhs.policy, policy);
    std::swap(rhs.descr, descr);
    std::swap(rhs.handle, handle);
    std::swap(rhs.infoTrans, infoTrans);
    std::swap(rhs.infoNoTrans, infoNoTrans);
    return *this;
}
SparseMatrix::CUDAPromoter::TriangularPromotion::operator bool() const
{
    return bufferTrans && bufferNoTrans;
}
SparseMatrix::CUDAPromoter::TriangularPromotion::TriangularPromotion(const SparseMatrix &m, bool upper, int gpuId)
{
    std::cout << "Triangular analysis" << std::endl;
    auto& promotion = m.gpuPromotion->basicPromotions[gpuId];
	std::cout << "Running TP on device id# " << gpuId << std::endl;
    cudaSetDevice(gpuId);
    checkError(__FILE__,  __LINE__);
    cusparseCreate(&handle);
    checkError(__FILE__,  __LINE__);
    cusparseCreateMatDescr(&descr);
    checkError(__FILE__,  __LINE__);
    cusparseSetMatIndexBase(descr, CUSPARSE_INDEX_BASE_ZERO);
    checkError(__FILE__,  __LINE__);
    cusparseSetMatFillMode(descr, upper ? CUSPARSE_FILL_MODE_UPPER : CUSPARSE_FILL_MODE_LOWER);
    checkError(__FILE__,  __LINE__);
    cusparseSetMatDiagType(descr, CUSPARSE_DIAG_TYPE_NON_UNIT);
    checkError(__FILE__,  __LINE__);

    cusparseCreateCsrsv2Info(&infoNoTrans);
    checkError(__FILE__,  __LINE__);

    double *values = promotion.dev_values.get();
    int *columns = promotion.dev_columns.get(), *rowPointers = promotion.dev_rowPointers.get(), nnz = m.nnz(), w = m.w;
    int bufferSize;
    void *pBuffer;

    auto op = CUSPARSE_OPERATION_NON_TRANSPOSE;
    cusparseDcsrsv2_bufferSize(handle, op, w, nnz, descr, values, rowPointers, columns, infoNoTrans, &bufferSize);
    checkError(__FILE__,  __LINE__);
    cudaMalloc(&pBuffer, bufferSize);
    checkError(__FILE__,  __LINE__);
    bufferNoTrans = GpuVoidPtr(pBuffer, cudaFree);
    cusparseDcsrsv2_analysis(handle, op, w, nnz, descr, values, rowPointers, columns, infoNoTrans, policy, pBuffer);
    checkError(__FILE__,  __LINE__);

    cusparseCreateCsrsv2Info(&infoTrans);
    checkError(__FILE__,  __LINE__);
    op = CUSPARSE_OPERATION_TRANSPOSE;
    cusparseDcsrsv2_bufferSize(handle, op, w, nnz, descr, values, rowPointers, columns, infoTrans, &bufferSize);
    checkError(__FILE__,  __LINE__);
    cudaMalloc(&pBuffer, bufferSize);
    checkError(__FILE__,  __LINE__);
    bufferTrans = GpuVoidPtr(pBuffer, cudaFree);
    cusparseDcsrsv2_analysis(handle, op, w, nnz, descr, values, rowPointers, columns, infoTrans, policy, pBuffer);
    checkError(__FILE__,  __LINE__);
    std::cout << "Level info created" << std::endl;
}

void SparseMatrix::CUDAPromoter::TriangularPromotion::checkError(const char* bar, int baz)
{
    auto err = cudaGetLastError();
    if (err != cudaSuccess)
    {
        std::cout <<  "CUDA FAILED: " << " at " << bar << ":" << baz << " "  << cudaGetErrorString(err) << std::flush;
    }
}

SparseMatrix::CUDAPromoter::TriangularPromotion::~TriangularPromotion()
{
    if (bufferTrans && bufferNoTrans)
    {
        cusparseDestroy(handle);
        cusparseDestroyMatDescr(descr);
        cusparseDestroyCsrsv2Info(infoTrans);
        cusparseDestroyCsrsv2Info(infoNoTrans);
    }
}

SparseMatrix::CUDAPromoter::BasicPromotion::BasicPromotion()
{
}

SparseMatrix::CUDAPromoter::BasicPromotion::BasicPromotion(const SparseMatrix &m, int gpuId)
{
	std::cout << "Running on device id# " << gpuId << std::endl;
    cudaSetDevice(gpuId);
    double *dv;
    std::cout << "Trying to allocate values  (" << m.nnz() * sizeof(double) / 1024.0 / 1024.0 << "Mb)" << std::endl;
    cudaMalloc(&(void*&)dv, m.nnz() * sizeof(double));
    dev_values = GpuDoublePtr(dv, cudaFree);
    SparseMatrix::CUDAPromoter::TriangularPromotion::checkError(__FILE__,  __LINE__);
    int *dc;
    std::cout << "Trying to allocate columns (" << m.nnz() * sizeof(int) / 1024.0 / 1024.0 << "Mb)" << std::endl;
    cudaMalloc(&(void*&)dc, m.nnz() * sizeof(int));
    dev_columns = GpuIntPtr(dc, cudaFree);
    SparseMatrix::CUDAPromoter::TriangularPromotion::checkError(__FILE__,  __LINE__);

    int *drp;
    std::cout << "Trying to allocate rowPointers (" << (m.h + 1) * sizeof(int) / 1024.0 / 1024.0 << "Mb)" << std::endl;
    cudaMalloc(&(void*&)drp, (m.h + 1) * sizeof(int));
    dev_rowPointers = GpuIntPtr(drp, cudaFree);
    SparseMatrix::CUDAPromoter::TriangularPromotion::checkError(__FILE__,  __LINE__);

    std::cout << "Copying values  (" << m.nnz() * sizeof(double) / 1024.0 / 1024.0 << "Mb)" << std::endl;
    cudaMemcpy(dv, &m.values     [0], m.nnz() * sizeof(double), cudaMemcpyHostToDevice);
    SparseMatrix::CUDAPromoter::TriangularPromotion::checkError(__FILE__,  __LINE__);
    std::cout << "Copying columns (" << m.nnz() * sizeof(int) / 1024.0 / 1024.0 << "Mb)" << std::endl;
    cudaMemcpy(dc, &m.columns    [0], m.nnz() * sizeof(int)   , cudaMemcpyHostToDevice);
    SparseMatrix::CUDAPromoter::TriangularPromotion::checkError(__FILE__,  __LINE__);
    std::cout << "Copying rowPointers (" << (m.h + 1) * sizeof(int) / 1024.0 / 1024.0 << "Mb)" << std::endl;
    cudaMemcpy(drp,&m.rowPointers[0], (m.h+1) * sizeof(int)   , cudaMemcpyHostToDevice);
    SparseMatrix::CUDAPromoter::TriangularPromotion::checkError(__FILE__,  __LINE__);
}

#if defined(_MSC_VER) && (_MSC_VER < 1900)   // needs only for older than msvc2015 compiler
SparseMatrix::CUDAPromoter::BasicPromotion& SparseMatrix::CUDAPromoter::BasicPromotion::operator=(SparseMatrix::CUDAPromoter::BasicPromotion &&rhs)
{
    std::swap(rhs.dev_values, dev_values);
    std::swap(rhs.dev_columns, dev_columns);
    std::swap(rhs.dev_rowPointers, dev_rowPointers);
    return *this;
}
#endif

SparseMatrix::CUDAPromoter::BasicPromotion::operator bool() const
{
    return dev_values && dev_columns && dev_rowPointers;
}

void SparseMatrix::CUDAPromoter::promote(int gpuId)
{
    if (!basicPromotions[gpuId])
    {
        std::cout << "Starting promotion to GPU#" << gpuId << std::endl;
        basicPromotions[gpuId] = BasicPromotion(*matrix, gpuId);
        std::cout << "Promoted" << std::endl;
    }
}

void SparseMatrix::CUDAPromoter::triangularAnalysis(bool upper, int gpuId)
{
    promote(gpuId);
    if (!triangularPromotions.count(gpuId))
        triangularPromotions[gpuId] = SparseMatrix::CUDAPromoter::TriangularPromotion(*matrix, upper, gpuId);
}
}

#endif
