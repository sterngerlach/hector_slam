
/* cma_memory.cpp */

#include "hw/cma_memory.hpp"
#include "hw/cma_manager.hpp"
#include "util/Assert.hpp"

namespace hectorslam {
namespace hw {

/*
 * CMAMemory class implementation
 */

/* Default constructor */
CMAMemory::CMAMemory() :
    mPtr(nullptr),
    mSize(0U),
    mPhysicalAddress(0U)
{
}

/* Constructor */
CMAMemory::CMAMemory(std::uint32_t lengthInBytes) :
    mPtr(nullptr),
    mSize(0U),
    mPhysicalAddress(0U)
{
    this->Initialize(lengthInBytes);
}

/* Destructor */
CMAMemory::~CMAMemory()
{
    /* Get the pointer to the CMA memory manager */
    CMAMemoryManager* const pCmaMgr = CMAMemoryManager::Instance();

    /* Free the CMA memory */
    if (pCmaMgr->IsLoaded() && this->mPtr != nullptr)
        pCmaMgr->CmaFree(this->mPtr);

    this->mPtr = nullptr;
    this->mSize = 0U;
    this->mPhysicalAddress = 0U;
}

/* Move constructor */
CMAMemory::CMAMemory(CMAMemory&& other) noexcept :
    mPtr(other.mPtr),
    mSize(other.mSize),
    mPhysicalAddress(other.mPhysicalAddress)
{
    other.mPtr = nullptr;
    other.mSize = 0U;
    other.mPhysicalAddress = 0U;
}

/* Move assignment operator */
CMAMemory& CMAMemory::operator=(CMAMemory&& other) noexcept
{
    if (this == &other)
        return *this;

    this->mPtr = other.mPtr;
    this->mSize = other.mSize;
    this->mPhysicalAddress = other.mPhysicalAddress;

    other.mPtr = nullptr;
    other.mSize = 0U;
    other.mPhysicalAddress = 0U;

    return *this;
}

/* Initialize the CMA memory */
void CMAMemory::Initialize(std::uint32_t lengthInBytes)
{
    /* Get the pointer to the CMA memory manager */
    CMAMemoryManager* const pCmaMgr = CMAMemoryManager::Instance();
    /* The CMA library should be loaded */
    XAssert(pCmaMgr->IsLoaded(), "CMA library (libcma.so) is not loaded");

    /* Allocate the CMA memory */
    this->mSize = lengthInBytes;
    this->mPtr = pCmaMgr->CmaAllocate(this->mSize);
    /* Get the physical address of the CMA memory */
    this->mPhysicalAddress = pCmaMgr->CmaGetPhysicalAddress(this->mPtr);
}

} /* namespace hw */
} /* namespace hectorslam */
