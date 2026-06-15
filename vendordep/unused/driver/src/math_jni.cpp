// Copyright (c) JSim contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the LGPLv3 license file in the root directory of this project.

#include "jni/math_jni.h"

#include <jni.h>

#include "frcsim/math/matrix.hpp"
#include "frcsim/math/quaternion.hpp"
#include "frcsim/math/vector.hpp"

#define JPTR(ptr) (reinterpret_cast<jlong>(ptr))
#define PTR(type, handle) (reinterpret_cast<type*>(handle))

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Class:     api_Vector3
 * Method:    nativeCreate
 * Signature: (DDD)J
 */
JNIEXPORT jlong JNICALL
Java_api_Vector3_nativeCreate
  (JNIEnv*, jobject, jdouble x, jdouble y, jdouble z)
{
  return JPTR(new frcsim::Vector3(x, y, z));
}
/*
 * Class:     api_Vector3
 * Method:    nativeNorm
 * Signature: (J)D
 */
JNIEXPORT jdouble JNICALL
Java_api_Vector3_nativeNorm
  (JNIEnv*, jobject, jlong ptr)
{
  return PTR(frcsim::Vector3, ptr)->norm();
}
/*
 * Class:     api_Vector3
 * Method:    nativeDelete
 * Signature: (J)V
 */
JNIEXPORT void JNICALL
Java_api_Vector3_nativeDelete
  (JNIEnv*, jobject, jlong ptr)
{
  delete PTR(frcsim::Vector3, ptr);
}

// Quaternion JNI
/*
 * Class:     api_Quaternion
 * Method:    nativeCreate
 * Signature: (DDDD)J
 */
JNIEXPORT jlong JNICALL
Java_api_Quaternion_nativeCreate
  (JNIEnv*, jobject, jdouble w, jdouble x, jdouble y, jdouble z)
{
  return JPTR(new frcsim::Quaternion(w, x, y, z));
}
/*
 * Class:     api_Quaternion
 * Method:    nativeFromAxisAngle
 * Signature: (JD)J
 */
JNIEXPORT jlong JNICALL
Java_api_Quaternion_nativeFromAxisAngle
  (JNIEnv*, jclass, jlong axisPtr, jdouble angle)
{
  auto axis = *PTR(frcsim::Vector3, axisPtr);
  return JPTR(new frcsim::Quaternion(frcsim::Quaternion::fromAxisAngle(axis, angle)));
}
/*
 * Class:     api_Quaternion
 * Method:    nativeMultiply
 * Signature: (JJ)J
 */
JNIEXPORT jlong JNICALL
Java_api_Quaternion_nativeMultiply
  (JNIEnv*, jobject, jlong ptrA, jlong ptrB)
{
  auto a = *PTR(frcsim::Quaternion, ptrA);
  auto b = *PTR(frcsim::Quaternion, ptrB);
  return JPTR(new frcsim::Quaternion(a * b));
}
/// Rotates vector v by quaternion q and returns a newly allocated result
/// Vector3.
/// @note The result is a heap allocation; the Java caller must invoke
/// nativeDelete.
/*
 * Class:     api_Quaternion
 * Method:    nativeRotate
 * Signature: (JJ)J
 */
JNIEXPORT jlong JNICALL
Java_api_Quaternion_nativeRotate
  (JNIEnv*, jobject, jlong qPtr, jlong vPtr)
{
  auto q = *PTR(frcsim::Quaternion, qPtr);
  auto v = *PTR(frcsim::Vector3, vPtr);
  auto result = q.rotate(v);
  return JPTR(new frcsim::Vector3(result));
}
/*
 * Class:     api_Quaternion
 * Method:    nativeDelete
 * Signature: (J)V
 */
JNIEXPORT void JNICALL
Java_api_Quaternion_nativeDelete
  (JNIEnv*, jobject, jlong ptr)
{
  delete PTR(frcsim::Quaternion, ptr);
}

// Matrix3 JNI
/*
 * Class:     api_Matrix3
 * Method:    nativeCreate
 * Signature: ()J
 */
JNIEXPORT jlong JNICALL
Java_api_Matrix3_nativeCreate
  (JNIEnv*, jobject)
{
  return JPTR(new frcsim::Matrix3());
}
/*
 * Class:     api_Matrix3
 * Method:    nativeMultiply
 * Signature: (JJ)J
 */
JNIEXPORT jlong JNICALL
Java_api_Matrix3_nativeMultiply
  (JNIEnv*, jobject, jlong mPtr, jlong vPtr)
{
  auto m = *PTR(frcsim::Matrix3, mPtr);
  auto v = *PTR(frcsim::Vector3, vPtr);
  auto result = m * v;
  return JPTR(new frcsim::Vector3(result));
}
/*
 * Class:     api_Matrix3
 * Method:    nativeDelete
 * Signature: (J)V
 */
JNIEXPORT void JNICALL
Java_api_Matrix3_nativeDelete
  (JNIEnv*, jobject, jlong ptr)
{
  delete PTR(frcsim::Matrix3, ptr);
}

#ifdef __cplusplus
}  // extern "C"
#endif
