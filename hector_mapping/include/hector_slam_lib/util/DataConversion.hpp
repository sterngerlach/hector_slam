
// DataConversion.hpp

#ifndef HECTOR_SLAM_UTIL_DATA_CONVERSION_HPP
#define HECTOR_SLAM_UTIL_DATA_CONVERSION_HPP

#include <cstdint>

namespace hectorslam {

// Union type for 32-bit data conversion
union U32Conversion
{
  std::int32_t  mI32Value;
  std::uint32_t mU32Value;
  float         mF32Value;
};

// Union type for 64-bit data conversion
union U64Conversion
{
  std::uint64_t mU64Value;
  std::uint8_t  mU8Values[8];

  struct
  {
    std::int32_t mValue0;
    std::int32_t mValue1;
  } mI32;

  struct
  {
    std::uint32_t mValue0;
    std::uint32_t mValue1;
  } mU32;

  struct
  {
    float mValue0;
    float mValue1;
  } mF32;
};

// Reinterpret a 32-bit signed integer value (int) as
// a 32-bit unsigned integer value (std::uint32_t)
inline std::uint32_t I32ToU32(const int value)
{
  U32Conversion dataConv;
  dataConv.mI32Value = value;
  return dataConv.mU32Value;
}

// Reinterpret a single-precision floating-point number (float) as
// a 32-bit unsigned integer value (std::uint32_t)
inline std::uint32_t FloatToU32(const float value)
{
  U32Conversion dataConv;
  dataConv.mF32Value = value;
  return dataConv.mU32Value;
}

// Reinterpret a 32-bit unsigned integer value (std::uint32_t) as
// a single-precision floating-point number (float)
inline float U32ToFloat(const std::uint32_t value)
{
  U32Conversion dataConv;
  dataConv.mU32Value = value;
  return dataConv.mF32Value;
}

// Pack two 32-bit unsigned integer values (std::uint32_t) and
// convert to a 64-bit unsigned integer value (std::uint64_t)
inline std::uint64_t PackU32(const std::uint32_t value0,
                             const std::uint32_t value1)
{
  U64Conversion dataConv;
  dataConv.mU32.mValue0 = value0;
  dataConv.mU32.mValue1 = value1;
  return dataConv.mU64Value;
}

// Pack two single-precision floating-point number (float) and
// convert to a 64-bit unsigned integer value (std::uint64_t)
inline std::uint64_t PackFloat(const float value0, const float value1)
{
  U64Conversion dataConv;
  dataConv.mF32.mValue0 = value0;
  dataConv.mF32.mValue1 = value1;
  return dataConv.mU64Value;
}

// Unpack a 64-bit unsigned integer value (std::uint64_t) and
// return two 32-bit signed integer values (int)
inline void UnpackI32(const std::uint64_t packedValue,
                      std::int32_t& value0, std::int32_t& value1)
{
  U64Conversion dataConv;
  dataConv.mU64Value = packedValue;
  value0 = dataConv.mI32.mValue0;
  value1 = dataConv.mI32.mValue1;
}

// Unpack a 64-bit unsigned integer value (std::uint64_t) and
// return two 32-bit unsigned integer values (std::uint32_t)
inline void UnpackU32(const std::uint64_t packedValue,
                      std::uint32_t& value0, std::uint32_t& value1)
{
  U64Conversion dataConv;
  dataConv.mU64Value = packedValue;
  value0 = dataConv.mU32.mValue0;
  value1 = dataConv.mU32.mValue1;
}

// Unpack a 64-bit unsigned integer value (std::uint64_t) and
// return two single-precision floating-point numbers (float)
inline void UnpackFloat(const std::uint64_t packedValue,
                        float& value0, float& value1)
{
  U64Conversion dataConv;
  dataConv.mU64Value = packedValue;
  value0 = dataConv.mF32.mValue0;
  value1 = dataConv.mF32.mValue1;
}

} // namespace hectorslam

#endif // HECTOR_SLAM_UTIL_DATA_CONVERSION_HPP
