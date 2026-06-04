/**
 * @file jogging_planes.h
 * @author Siddhi Jain (siddhi.jain@addverb.com)
 * @brief Defined enums for safety planes of jogging
 * @version 0.1
 * @date 2024-08-21
 *
 * @copyright Copyright (c) 2024
 *
 */

#ifndef CART_JOGGING_PLANES_H_
#define CART_JOGGING_PLANES_H_

/// @brief list of identifiers for safety planes that can be encountered when in Cartesain jogging modes
enum class CartJoggingPlanes
{
    eNone = 0,
    eUpperPlane = 1,
    eLowerPlane = 2,
    eInnerCylinder = 3,
    eOuterCylinder = 4

};

#endif