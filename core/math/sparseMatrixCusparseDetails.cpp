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
    cudaSetDevice(gpuId);
    checkError();
    cusparseCreate(&handle);
    checkError();
    cusparseCreateMatDescr(&descr);
    checkError();
    cusparseSetMatIndexBase(descr, CUSPARSE_INDEX_BASE_ZERO);
    checkError();
    cusparseSetMatFillMode(descr, upper ? CUSPARSE_FILL_MODE_UPPER : CUSPARSE_FILL_MODE_LOWER);
    checkError();
    cusparseSetMatDiagType(descr, CUSPARSE_DIAG_TYPE_NON_UNIT);
    checkError();

    cusparseCreateCsrsv2Info(&infoNoTrans);
    checkError();

    double *values = promotion.dev_values.get();
    int *columns = promotion.dev_columns.get(), *rowPointers = promotion.dev_rowPointers.get(), nnz = m.nnz(), w = m.w;
    int bufferSize;
    void *pBuffer;

    auto op = CUSPARSE_OPERATION_NON_TRANSPOSE;
    cusparseDcsrsv2_bufferSize(handle, op, w, nnz, descr, values, rowPointers, columns, infoNoTrans, &bufferSize);
    checkError();
    cudaMalloc(&pBuffer, bufferSize);
    checkError();
    bufferNoTrans = GpuVoidPtr(pBuffer, cudaFree);
    cusparseDcsrsv2_analysis(handle, op, w, nnz, descr, values, rowPointers, columns, infoNoTrans, policy, pBuffer);
    checkError();

    cusparseCreateCsrsv2Info(&infoTrans);
    checkError();
    op = CUSPARSE_OPERATION_TRANSPOSE;
    cusparseDcsrsv2_bufferSize(handle, op, w, nnz, descr, values, rowPointers, columns, infoTrans, &bufferSize);
    checkError();
    cudaMalloc(&pBuffer, bufferSize);
    checkError();
    bufferTrans = GpuVoidPtr(pBuffer, cudaFree);
    cusparseDcsrsv2_analysis(handle, op, w, nnz, descr, values, rowPointers, columns, infoTrans, policy, pBuffer);
    checkError();
    std::cout << "Level info created" << std::endl;
}

void SparseMatrix::CUDAPromoter::TriangularPromotion::checkError()
{
    auto err = cudaGetLastError();
    if (err != cudaSuccess)
    {
        std::cout <<  "CUDA FAILED: " << cudaGetErrorString(err);
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
    cudaSetDevice(gpuId);
    double *dv;
    std::cout << "Trying to allocate values  (" << m.nnz() * sizeof(double) / 1024.0 / 1024.0 << "Mb)" << std::endl;
    cudaMalloc(&(void*&)dv, m.nnz() * sizeof(double));
    dev_values = GpuDoublePtr(dv, cudaFree);
    SparseMatrix::CUDAPromoter::TriangularPromotion::checkError();
    int *dc;
    std::cout << "Trying to allocate columns (" << m.nnz() * sizeof(int) / 1024.0 / 1024.0 << "Mb)" << std::endl;
    cudaMalloc(&(void*&)dc, m.nnz() * sizeof(int));
    dev_columns = GpuIntPtr(dc, cudaFree);
    SparseMatrix::CUDAPromoter::TriangularPromotion::checkError();

    int *drp;
    std::cout << "Trying to allocate rowPointers (" << (m.h + 1) * sizeof(int) / 1024.0 / 1024.0 << "Mb)" << std::endl;
    cudaMalloc(&(void*&)drp, (m.h + 1) * sizeof(int));
    dev_rowPointers = GpuIntPtr(drp, cudaFree);
    SparseMatrix::CUDAPromoter::TriangularPromotion::checkError();

    std::cout << "Copying values  (" << m.nnz() * sizeof(double) / 1024.0 / 1024.0 << "Mb)" << std::endl;
    cudaMemcpy(dv, &m.values     [0], m.nnz() * sizeof(double), cudaMemcpyHostToDevice);
    SparseMatrix::CUDAPromoter::TriangularPromotion::checkError();
    std::cout << "Copying columns (" << m.nnz() * sizeof(int) / 1024.0 / 1024.0 << "Mb)" << std::endl;
    cudaMemcpy(dc, &m.columns    [0], m.nnz() * sizeof(int)   , cudaMemcpyHostToDevice);
    SparseMatrix::CUDAPromoter::TriangularPromotion::checkError();
    std::cout << "Copying rowPointers (" << (m.h + 1) * sizeof(int) / 1024.0 / 1024.0 << "Mb)" << std::endl;
    cudaMemcpy(drp,&m.rowPointers[0], (m.h+1) * sizeof(int)   , cudaMemcpyHostToDevice);
    SparseMatrix::CUDAPromoter::TriangularPromotion::checkError();
}

SparseMatrix::CUDAPromoter::BasicPromotion& SparseMatrix::CUDAPromoter::BasicPromotion::operator=(SparseMatrix::CUDAPromoter::BasicPromotion &&rhs)
{
    std::swap(rhs.dev_values, dev_values);
    std::swap(rhs.dev_columns, dev_columns);
    std::swap(rhs.dev_rowPointers, dev_rowPointers);
    return *this;
}

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
