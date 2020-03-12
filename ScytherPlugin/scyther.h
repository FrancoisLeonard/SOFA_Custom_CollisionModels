#ifndef SOFA_PLUGIN_SCYTHER_H
#define SOFA_PLUGIN_SCYTHER_H

#include <sofa/helper/system/config.h>

#ifdef SOFA_BUILD_SCYTHER_PLUGIN
#  define SOFA_SCYTHER_API SOFA_EXPORT_DYNAMIC_LIBRARY
#else
#  define SOFA_SCYTHER_API SOFA_IMPORT_DYNAMIC_LIBRARY
#endif

namespace sofa
{

namespace remote
{

enum SerializationFormat { RAW = 0, JSON = 1 };

} // namespace remote

} // namespace sofa


#endif /* SOFA_PLUGIN_SOFASSR_H */