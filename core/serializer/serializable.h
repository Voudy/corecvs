#ifndef SERIALIZABLE_H_
#define SERIALIZABLE_H_
/**
 * \file serializable.h
 * \brief Add Comment Here
 *
 * \date May 22, 2011
 * \author alexander
 */

#include <string>
#include <vector>
#include <core/utils/propertyList.h>

namespace corecvs {

using std::string;

class Serializer
{

template<typename ValueType>
    int visit(ValueType &value, string name, const ValueType defaultValue);

    int startGroup(string name);
    int endGroup();
};

template<typename RealType>
class Serializable
{
public:
    void serialize();
};

class PropertyListSerializer : public PropertyList
{

template<typename ValueType>
    int visit(ValueType &value, string name, const ValueType defaultValue);

};


} //namespace corecvs
#endif /* SERIALIZABLE_H_ */

