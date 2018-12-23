/**
 * \file serializable.cpp
 * \brief Add Comment Here
 *
 * \date May 22, 2011
 * \author alexander
 */

#include "core/serializer/serializable.h"
namespace corecvs {

template<typename ValueType>
int PropertyListSerializer::visit(ValueType &value, std::string name, const ValueType defaultValue)
{
    return true;
}

} //namespace corecvs

